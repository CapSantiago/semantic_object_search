"""Browser console contracts without ROS, TCP sockets, or real robot clients."""
import io
import json
import signal
import subprocess
import threading
from types import SimpleNamespace

import pytest

import robot_console as module


class Child:
    def __init__(self):
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired('fake-client', timeout)
        return self.returncode


class Runtime:
    def __init__(self, directory):
        self.session_dir = directory / 'session'
        self.session_dir.mkdir()
        (self.session_dir / 'tasks').mkdir()
        self.headless = True
        self.calls, self.signals = [], []
        self.failure = None
        self.closed = False
        self.rows = {name: {'state': 'running', 'pid': None, 'returncode': None, 'log': None}
                     for name in ('simulation', 'navigation', 'detector', 'search')}

    def snapshot(self):
        return dict(self.rows)

    def start_process(self, name, argv):
        if self.failure:
            raise self.failure
        child = Child()
        self.calls.append((name, argv, child))
        return child

    def signal_process(self, name, sig=signal.SIGINT):
        self.signals.append((name, sig))
        return True

    def log_tail(self, name):
        raise KeyError(name)

    def close(self):
        self.closed = True


class Monitor:
    def __init__(self):
        self.values = dict(simulation_ready=True, navigation_ready=True, perception_ready=True)
        self.closed = False

    def snapshot(self):
        return dict(self.values)

    def map_snapshot(self):
        return {'available': False}

    def close(self):
        self.closed = True


@pytest.fixture
def console(tmp_path, monkeypatch):
    # Defer only the observer thread. Production submit/cancel/result handlers
    # still parse actual commands and read/write evidence in a temporary session.
    workers = []

    class PendingThread:
        def __init__(self, target, args=(), **unused):
            self.target, self.args = target, args
            self.finished = False

        def start(self):
            workers.append(self)

        def run(self):
            self.target(*self.args)
            self.finished = True

        def join(self, timeout=None):
            if not self.finished:
                self.run()

        def is_alive(self):
            return not self.finished

    monkeypatch.setattr(module, 'threading', SimpleNamespace(
        RLock=threading.RLock, Event=threading.Event, Thread=PendingThread))
    value = module.Console(Runtime(tmp_path), Monitor())
    value.phase = 'ready'
    value.pending_workers = workers
    return value


def task(request_id='request_001', **changes):
    value = dict(request_id=request_id, mode='find', room='kitchen', category='cup')
    value.update(changes)
    return value


def put_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def finish_search(console, status='found', code=0, task_id='task_001',extra=None):
    folder = console.job['_folder']
    result = dict(status=status, count=1 if status == 'found' else 0, objects=[])
    result.update(extra or {})
    if task_id is not None:
        result['task_id'] = task_id
        persisted = console.runtime.session_dir / 'tasks' / task_id
        put_json(persisted / 'result.json', result)
        put_json(persisted / 'observations.json', [])
        (persisted / 'semantic_map.svg').write_text('<svg/>')
    put_json(folder / 'client_result.json', result)
    put_json(folder / 'diagnostics.json', {'command_sent': True, 'errors': []})
    put_json(folder / 'initial_state.json', {'frame': 'map', 'x': 0, 'y': 0})
    console.runtime.calls[-1][2].returncode = code
    console.pending_workers[-1].run()
    return result


def station_enabled(console,monkeypatch,*,return_status='completed',settled=True,cancel_at_start=False):
    console.charging_station=dict(id='home_dock',display_name='家庭充电处',frame_id='map',
                                  map_pose=[-1.8,-4.,0.],simulation_only=True)
    console.charging_station_file=console.runtime.session_dir/'station.json'
    original=console.runtime.start_process
    def start(name,argv):
        child=original(name,argv)
        if name.startswith('return_'):
            folder=console.job['_folder']
            assert all((console.job['_persisted_result'].parent/p).is_file() for p in
                       ('result.json','observations.json','semantic_map.svg'))
            assert console.snapshot()['busy']
            assert_api_error(409,lambda:console.submit(task(request_id='second_busy_01')))
            if cancel_at_start:console.cancel()
            result=dict(status=return_status,navigation_settled=settled,
                return_state='simulated_charging' if return_status=='completed' else 'return_incomplete',
                return_error=dict(position_m=.05,yaw_rad=.06,stationary_seconds=.46),message='return '+return_status)
            put_json(folder/'return_result.json',result);put_json(folder/'return_trace.json',{'goals':[]})
            child.returncode=0 if return_status=='completed' else 130 if return_status=='canceled' else 1
        return child
    monkeypatch.setattr(console.runtime,'start_process',start)


@pytest.mark.parametrize('return_status,settled,expected',[
    ('completed',True,'completed'),('partial',True,'partial'),('failed',False,'failed')])
def test_search_persists_results_before_return_and_keeps_them_on_return_failure(console,monkeypatch,return_status,settled,expected):
    station_enabled(console,monkeypatch,return_status=return_status,settled=settled)
    console.submit(task());result=finish_search(console)
    public=console.snapshot()['task']
    assert public['state']==expected and public['result']==result
    assert public['save_status']=='saved'
    assert public['memory'] and any(x['name']=='返航报告' for x in public['artifacts'])
    assert console.runtime.calls[-1][0].startswith('return_')
    assert json.loads((console.job['_persisted_result']).read_text())==result
    assert console.uncertain is (not settled)
    assert console.runtime.closed is (not settled)


def test_cancel_while_saving_does_not_launch_return_or_discard_primary_result(console,monkeypatch):
    station_enabled(console,monkeypatch)
    def remember(row,result):
        console.cancel()
        return {'status':'saved'}
    monkeypatch.setattr(console,'_remember_search',remember)
    console.submit(task());result=finish_search(console)
    public=console.snapshot()['task']
    assert public['state']=='canceled' and public['result']==result
    assert public['return']['state']=='skipped_canceled'
    assert public['save_status']=='saved' and len(console.runtime.calls)==1


def test_cancel_at_return_start_targets_the_return_process_and_keeps_result(console,monkeypatch):
    station_enabled(console,monkeypatch,return_status='canceled',cancel_at_start=True)
    console.submit(task());result=finish_search(console)
    public=console.snapshot()['task']
    assert public['state']=='canceled' and public['result']==result
    assert console.runtime.signals[-1][0].startswith('return_')
    assert not console.uncertain


def test_missing_persisted_observations_never_starts_automatic_return(console,monkeypatch):
    station_enabled(console,monkeypatch)
    original=console._finish_artifacts
    def finish(row,result):
        artifacts,status=original(row,result)
        (row['_persisted_result'].parent/'observations.json').unlink()
        return artifacts,'partial'
    monkeypatch.setattr(console,'_finish_artifacts',finish)
    console.submit(task());result=finish_search(console)
    public=console.snapshot()['task']
    assert public['state']=='partial' and public['result']==result
    assert public['return']['state']=='skipped_unpersisted'
    assert len(console.runtime.calls)==1


def test_explicit_return_command_uses_the_same_managed_client_and_receipt(console):
    console.charging_station=dict(id='home_dock',display_name='充电处',frame_id='map',
                                  map_pose=[-1.8,-4.,0.],simulation_only=True)
    console.charging_station_file=console.runtime.session_dir/'station.json'
    console.submit({'request_id':'manual_return_01','command':'返回充电处'})
    assert console.job['kind']=='return'
    assert console.runtime.calls[-1][1][1].endswith('/run_return.py')
    folder=console.job['_folder'];result=dict(status='completed',navigation_settled=True,
        return_state='simulated_charging',message='done')
    put_json(folder/'return_result.json',result);put_json(folder/'return_trace.json',{'goals':[]})
    console.runtime.calls[-1][2].returncode=0;console.pending_workers[-1].run()
    assert console.snapshot()['task']['state']=='completed'


def assert_api_error(status, action):
    with pytest.raises(module.ApiError) as caught:
        action()
    assert caught.value.status == status


@pytest.mark.parametrize('sample_grid,artifact_exists,expected',[
    (False,False,'saved'),(True,False,'partial'),(True,True,'saved')])
def test_early_cancel_requires_coverage_artifact_only_after_grid_creation(console,sample_grid,artifact_exists,expected):
    console.submit(task())
    geometry = dict(samples=10) if sample_grid else dict(stop_reason='no_valid_geometry')
    if artifact_exists:
        put_json(console.runtime.session_dir/'tasks/task_001/coverage_samples.json',{})
    finish_search(console,status='canceled',code=130,extra=dict(geometric_coverage=geometry))
    assert console.snapshot()['task']['save_status']==expected


def test_command_normalization_uses_existing_room_and_object_parser(console):
    _, _, structured = console.normalize(task())
    request_id, original, natural = console.normalize(
        {'request_id': 'request_001', 'command': '  帮我去厨房找杯子  '})
    assert request_id == 'request_001'
    assert original == '帮我去厨房找杯子'
    assert json.loads(structured) == dict(mode='find', room='kitchen', category='cup')
    assert natural == structured
    assert json.loads(console.normalize(task(mode='count'))[2])['mode'] == 'count'


def test_catalog_has_sixteen_targets_but_only_admitted_tasks_are_callable(console):
    catalog = console.snapshot()['object_catalog']
    assert len(catalog) == 16
    assert {item['category'] for item in catalog if item['admitted']} == {'cup', 'bottle', 'chair'}
    assert_api_error(400, lambda: console.normalize(task(category='bowl')))
    assert json.loads(console.normalize({'request_id':'request_001','command':'去厨房找马克杯'})[2])['category'] == 'cup'


def test_candidate_enablement_does_not_change_admission_or_other_categories(console):
    console.target_categories = ('cup','bottle','chair','bowl','cell phone')
    catalog = {row['category']:row for row in console.snapshot()['object_catalog']}
    assert catalog['bowl']['validation_enabled'] and not catalog['bowl']['admitted']
    assert not catalog['book']['validation_enabled']
    assert json.loads(console.normalize(task(category='bowl'))[2])['category'] == 'bowl'
    assert json.loads(console.normalize({'request_id':'request_001','command':'去厨房找手机'})[2])['category'] == 'cell phone'
    assert_api_error(400, lambda:console.normalize(task(category='book')))


def test_memory_query_is_scoped_to_current_map_and_mode(console):
    from semantic_search.semantic_memory import map_namespace
    grid = dict(available=True, frame_id='map', width=2, height=2, resolution=.05,
                origin=dict(x=0., y=0., yaw=0.), data=[0, 0, -1, 100])
    console.monitor.map_snapshot = lambda: dict(grid)
    value = dict(task_id='search-memory', category='cup', room='kitchen', mode='find', status='found',
                 frame_id='map', coordinate_unit='m', count=1, objects=[dict(id=1,x=0.,y=0.,z=.8,hits=3)])
    console.memory.ingest(map_namespace(grid, console.rooms), value,
                         observed_at='2026-11-21T10:00:00+08:00', evidence='tasks/source/result.json')
    response = console.memory_snapshot(room='kitchen', category='cup')
    assert response['available'] and response['total'] == 1
    assert console.memory_snapshot(category='bottle')['total'] == 0
    assert_api_error(400, lambda: console.memory_snapshot(category='dumbbell'))
    grid['origin'] = dict(x=1., y=0., yaw=0.)
    assert console.memory_snapshot()['total'] == 0
    console.mode = 'mapping'
    assert not console.memory_snapshot()['available']


def test_completed_search_imports_only_matching_persisted_evidence(console):
    console.monitor.map_snapshot = lambda: dict(available=True, frame_id='map', width=2, height=2,
        resolution=.05, origin=dict(x=0.,y=0.,yaw=0.), data=[0,0,-1,100])
    console.submit(task())
    folder = console.job['_folder']
    value = dict(task_id='search-memory', category='cup', room='kitchen', mode='find', status='found',
                 frame_id='map', coordinate_unit='m', count=1, objects=[dict(id=1,x=0.,y=0.,z=.8,hits=3)])
    source = console.runtime.session_dir/'tasks/search-memory/result.json'
    put_json(source, value)
    put_json(source.with_name('observations.json'), [])
    source.with_name('semantic_map.svg').write_text('<svg/>')
    put_json(folder/'client_result.json', value)
    put_json(folder/'diagnostics.json', dict(command_sent=True, errors=[]))
    put_json(folder/'initial_state.json', {})
    console.runtime.calls[-1][2].returncode = 0
    console.pending_workers[-1].run()
    assert console.job['state'] == 'completed'
    assert console.job['memory']['status'] == 'saved'
    assert console.memory_snapshot()['total'] == 1
    source.write_text('{}')
    rejected = console._remember_search(console.job, value)
    assert rejected['status'] == 'failed'
    assert console.memory_snapshot()['total'] == 1


def test_progress_is_visible_only_for_the_matching_request(console):
    first = console.submit(task())['task']
    event = dict(request_id=first['id'], task_id='search_001', stage='observing', count=1)
    console.monitor.values['search_progress'] = event
    assert console.snapshot()['task']['progress'] == event
    finish_search(console)
    next_task = console.submit(task('request_002'))['task']
    assert 'progress' not in next_task


@pytest.mark.parametrize('position_goals',[False,True])
@pytest.mark.parametrize('local_recovery',[False,True])
@pytest.mark.parametrize('budget_selection',[False,True])
def test_cleaning_shares_request_lock_and_persists_its_own_results(console,position_goals,local_recovery,budget_selection):
    console.cleaning_position_goals = position_goals
    console.cleaning_local_recovery = local_recovery
    console.cleaning_budget_selection = budget_selection
    request = dict(request_id='clean_room_01', mode='clean', room='gym')
    submitted = console.submit(request)
    assert submitted['busy'] and submitted['task']['kind']=='clean'
    assert '/run_cleaning.py' in console.runtime.calls[-1][1][1]
    assert ('--position-goals' in console.runtime.calls[-1][1])==position_goals
    assert ('--local-recovery' in console.runtime.calls[-1][1])==local_recovery
    assert ('--budget-selection' in console.runtime.calls[-1][1])==budget_selection
    assert console.submit(dict(request_id='clean_room_01',command='去运动区扫地'))['busy']
    assert len(console.runtime.calls)==1
    assert_api_error(409, lambda: console.submit(task()))
    folder = console.job['_folder']
    put_json(folder/'clean_progress.json',dict(stage='sweeping'))
    assert console.snapshot()['task']['progress']['stage']=='sweeping'
    put_json(folder/'clean_result.json',dict(status='completed',navigation_settled=True,coverage={'coverage_fraction':.9}))
    put_json(folder/'clean_trace.json',[])
    (folder/'clean_map.svg').write_text('<svg/>')
    console.runtime.calls[-1][2].returncode = 0
    console.pending_workers[-1].run()
    result = console.snapshot()['task']
    assert result['state']=='completed' and result['save_status']=='saved'
    assert len(result['artifacts'])==4


def test_cleaning_with_unknown_action_state_closes_owned_navigation(console):
    console.submit(dict(request_id='clean_room_01',mode='clean',room='gym'))
    put_json(console.job['_folder']/'clean_result.json',dict(status='failed',navigation_settled=False))
    console.runtime.calls[-1][2].returncode = 1
    console.pending_workers[-1].run()
    assert console.snapshot()['control_uncertain']
    assert console.runtime.closed
    assert_api_error(409, lambda: console.submit(task()))


def test_voice_returns_editable_text_without_sending_a_search(console):
    console.voice_available = True
    pending = console.transcribe(b'fixture recording')
    assert pending['busy'] and pending['task']['kind'] == 'voice'
    assert_api_error(409, lambda: console.submit(task()))
    folder = console.job['_folder']
    assert (folder/'recording.audio').is_file()
    put_json(folder/'transcript.json', {'text': '去厨房找杯子'})
    console.runtime.calls[-1][2].returncode = 0
    console.pending_workers[-1].run()
    final = console.snapshot()
    assert final['task']['result']['text'] == '去厨房找杯子'
    assert final['ready'] and len(console.runtime.calls) == 1
    assert not (folder/'recording.audio').exists()


@pytest.mark.parametrize('payload', [
    None, [], {'request_id': '../unsafe', 'command': '去厨房找杯子'},
    {'request_id': 'request_001', 'command': '停止'},
    {'request_id': 'request_001', 'command': ''},
    {'request_id': 'request_001', 'command': '找' * 501},
    {'request_id': 'request_001', 'command': ['去厨房找杯子']},
    {'request_id': 'request_001', 'command': '去厨房找杯子', 'shell': 'echo unsafe'},
    task(room='garage'), task(category='knife'), task(mode='cancel'),
    task(room=['kitchen']), task(extra='unexpected'),
])
def test_invalid_task_never_launches_a_process(console, payload):
    assert_api_error(400, lambda: console.submit(payload))
    assert not console.runtime.calls
    assert not (console.runtime.session_dir / 'requests').exists()


def test_single_task_and_retry_idempotency_survive_completion(console):
    first = console.submit(task())
    assert first['busy'] and not first['ready']
    retried = console.submit({'request_id': 'request_001', 'command': '去厨房找杯子'})
    assert retried['task']['id'] == first['task']['id']
    assert len(console.runtime.calls) == 1
    assert_api_error(409, lambda: console.submit(task('request_002')))
    assert_api_error(409, lambda: console.submit(task(category='bottle')))
    assert_api_error(409, console.validate)
    saved = json.loads((console.job['_folder'] / 'request.json').read_text())
    assert json.loads(saved['executed_command']) == dict(
        mode='find', room='kitchen', category='cup', request_id='request_001')
    finish_search(console)
    completed = console.submit(task())
    assert completed['task']['state'] == 'completed'
    assert len(console.runtime.calls) == 1
    console.submit(task('request_002'))
    argv = console.runtime.calls[-1][1]
    assert argv[argv.index('--exclude-task-id') + 1] == 'task_001'


@pytest.mark.parametrize('status,code,state', [
    ('found', 0, 'completed'), ('not_found', 1, 'completed'),
    ('completed', 0, 'completed'), ('partial', 1, 'partial'),
])
def test_results_and_downloadable_evidence_are_preserved(console, status, code, state):
    console.submit(task())
    result = finish_search(console, status, code)
    view = console.snapshot()
    row = view['task']
    assert row['state'] == state
    assert row['result'] == result
    assert row['save_status'] == 'saved'
    assert not view['busy'] and view['ready']
    assert len(row['artifacts']) == 7
    assert all(not key.startswith('_') for key in row)
    assert all(path.is_relative_to(console.runtime.session_dir) for path in console.artifacts.values())
    receipt = json.loads((console.job['_folder'] / 'console_result.json').read_text())
    assert receipt['state'] == state and receipt['result'] == result
    assert view['history'][0]['id'] == row['id']


def test_uncertain_client_exit_blocks_new_motion(console):
    console.submit(task())
    put_json(console.job['_folder'] / 'diagnostics.json', {'command_sent': True})
    console.runtime.calls[-1][2].returncode = 2
    console.pending_workers[-1].run()
    view = console.snapshot()
    assert view['task']['state'] == 'failed'
    assert view['control_uncertain'] and not view['ready']
    assert view['task']['save_status'] == 'partial'
    assert_api_error(409, lambda: console.submit(task('request_002')))
    assert len(console.runtime.calls) == 1


def test_corrupt_result_is_reported_without_claiming_completion(console):
    console.submit(task())
    (console.job['_folder'] / 'client_result.json').write_text('{broken')
    console.runtime.calls[-1][2].returncode = 0
    console.pending_workers[-1].run()
    assert console.job['state'] == 'failed'
    assert console.job['save_status'] == 'missing'
    assert console.snapshot()['control_uncertain']
    assert json.loads((console.job['_folder'] / 'console_result.json').read_text())['error']


def test_process_start_failure_is_terminal_and_records_request(console):
    console.runtime.failure = OSError('client executable unavailable')
    assert_api_error(500, lambda: console.submit(task()))
    view = console.snapshot()
    assert view['task']['state'] == 'failed'
    assert not view['busy']
    assert 'client executable unavailable' in view['task']['error']
    assert (console.job['_folder'] / 'request.json').is_file()
    assert not console.runtime.calls
    assert view['control_uncertain'] and console.runtime.closed


def test_cancel_only_signals_active_client_and_is_idempotent(console):
    console.submit(task())
    first = console.cancel()
    assert first['task']['state'] == 'canceling'
    console.cancel()
    assert console.runtime.signals == [(console.runtime.calls[0][0], signal.SIGINT)]
    assert not console.runtime.closed
    finish_search(console, status='canceled', code=130, task_id=None)
    view = console.cancel()
    assert view['task']['state'] == 'canceled'
    assert not view['control_uncertain']
    assert len(console.runtime.signals) == 1


def test_validation_saves_sensor_report_and_sample(console):
    view = console.validate()
    assert view['validation']['state'] == 'running'
    assert console.runtime.calls[-1][1][-2] == '--output'
    sensors = console.job['_folder'] / 'sensors'
    put_json(sensors / 'sensors.json', {'passed': True})
    (sensors / 'rgb.png').write_bytes(b'test image')
    console.runtime.calls[-1][2].returncode = 0
    console.pending_workers[-1].run()
    view = console.snapshot()
    assert view['validation']['state'] == 'completed'
    assert view['validation']['save_status'] == 'saved'
    assert len(view['validation']['artifacts']) == 3


@pytest.mark.parametrize('issue', ['phase', 'sensors', 'component'])
def test_incomplete_readiness_rejects_work(console, issue):
    if issue == 'phase':
        console.phase = 'navigation'
    elif issue == 'sensors':
        console.monitor.values['perception_ready'] = False
    else:
        console.runtime.rows['search']['state'] = 'exited'
    assert not console.snapshot()['ready']
    assert_api_error(409, lambda: console.submit(task()))
    assert not console.runtime.calls


def http_request(console, method='GET', path='/api/state', body=b'', headers=None):
    """Pass real HTTP bytes through BaseHTTPRequestHandler over memory streams."""
    values = {'Host': '127.0.0.1:8765', 'Connection': 'close'}
    if method == 'POST':
        values.update({'Content-Type': 'application/json', 'Content-Length': str(len(body))})
    values.update(headers or {})
    raw = (method + ' ' + path + ' HTTP/1.1\r\n' + ''.join(
        key + ': ' + value + '\r\n' for key, value in values.items()) + '\r\n').encode() + body

    class Connection:
        def __init__(self):
            self.response = bytearray()

        def makefile(self, *unused):
            return io.BytesIO(raw)

        def sendall(self, value):
            self.response.extend(value)

    connection = Connection()
    module.handler_for(console)(connection, ('127.0.0.1', 12345), SimpleNamespace(server_port=8765))
    head, payload = bytes(connection.response).split(b'\r\n\r\n', 1)
    lines = head.decode().split('\r\n')
    return int(lines[0].split()[1]), dict(line.split(': ', 1) for line in lines[1:]), payload


@pytest.mark.parametrize('headers', [
    {'Host': 'attacker.example:8765'}, {'Host': '127.0.0.1:8000'},
    {'Origin': 'https://attacker.example'}, {'Origin': 'null'},
])
def test_http_rejects_foreign_host_or_origin(console, headers):
    assert http_request(console, headers=headers)[0] == 403
    status, _, _ = http_request(console, method='POST', path='/api/task',
                               body=json.dumps(task()).encode(), headers=headers)
    assert status == 403 and not console.runtime.calls


def test_http_serves_state_and_accepts_local_json_task(console):
    status, headers, body = http_request(console, headers={'Origin': 'http://localhost:8765'})
    assert status == 200 and json.loads(body)['ready']
    assert headers['X-Content-Type-Options'] == 'nosniff'
    assert headers['Cache-Control'] == 'no-store'
    assert "frame-ancestors 'none'" in headers['Content-Security-Policy']
    status, _, body = http_request(console, method='POST', path='/api/task',
                                 body=json.dumps(task()).encode())
    assert status == 200 and json.loads(body)['busy']
    status, _, _ = http_request(console, method='POST', path='/api/task',
                               body=json.dumps(task()).encode())
    assert status == 200 and len(console.runtime.calls) == 1


def test_memory_http_contract_waits_for_map_and_rejects_unknown_filters(console):
    status, _, payload = http_request(console,path='/api/memory')
    assert status == 200 and not json.loads(payload)['available']
    assert http_request(console,path='/api/memory?category=dumbbell')[0] == 400
    assert http_request(console,path='/api/memory/graph')[0] == 409
    assert http_request(console,path='/api/memory',headers={'Origin':'https://attacker.example'})[0] == 403


@pytest.mark.parametrize('body,headers,status', [
    (b'{}', {'Content-Type': 'text/plain'}, 415),
    (b'[]', {}, 400), (b'{invalid', {}, 400),
    (b'{}', {'Content-Length': '5000'}, 413),
    (b'{}', {'Content-Length': '-1'}, 413),
    (b'{}', {'Content-Length': 'invalid'}, 400),
])
def test_http_rejects_malformed_control_requests(console, body, headers, status):
    actual, _, _ = http_request(console, method='POST', path='/api/task', body=body, headers=headers)
    assert actual == status and not console.runtime.calls


def test_http_artifacts_are_allowlisted_and_rechecked_after_symlink_changes(console, tmp_path):
    outside = tmp_path / 'secret.txt'
    outside.write_text('outside session')
    assert console._register(outside, 'forbidden') is None
    link = console.runtime.session_dir / 'external.txt'
    link.symlink_to(outside)
    assert console._register(link, 'forbidden link') is None
    target = console.runtime.session_dir / 'output.json'
    target.write_text('{"saved":true}')
    registered = console._register(target, 'result')
    status, _, body = http_request(console, path=registered['url'])
    assert status == 200 and json.loads(body)['saved']
    for path in ('/api/artifact?id=../secret.txt', '/api/artifact?path=' + str(outside),
                 '/../secret.txt', '/%2e%2e/secret.txt'):
        assert http_request(console, path=path)[0] == 404
    target.unlink()
    target.symlink_to(outside)
    assert http_request(console, path=registered['url'])[0] == 404


def test_http_rejects_unknown_log_component(console):
    assert http_request(console, path='/api/log?component=../../secret')[0] == 400
    status, _, body = http_request(console, path='/api/log?component=rviz')
    assert status == 200 and isinstance(json.loads(body)['text'], str)


def test_room_overlay_uses_the_same_config_as_command_parsing(console):
    state = console.snapshot()
    assert state['rooms_frame_id'] == 'map'
    assert set(state['rooms']) == {'kitchen','living','bedroom','utility','gym'}
    assert state['rooms']['gym']['display_name'] == '健身区'
    _, _, command = console.normalize({'request_id':'gym_command_1','command':'去健身区找椅子'})
    assert json.loads(command)['room'] == 'gym'


def test_costmap_export_is_read_only_allowlisted_and_rejects_expired_data(console,monkeypatch):
    grid=dict(available=True,frame_id='map',width=2,height=2,resolution=1.,
              origin=dict(x=0.,y=0.,yaw=0.),data=[0,0,0,0])
    costs=dict(grid,source='/global_costmap/costmap',fresh=True,stamp_seconds=10.)
    monkeypatch.setattr(console.monitor,'map_snapshot',lambda:dict(grid))
    monkeypatch.setattr(console.monitor,'costmap_snapshot',lambda:dict(costs),raising=False)
    console.monitor.values['clock_seconds']=10.5
    status,_,body=http_request(console,method='POST',path='/api/map/export',body=b'{}')
    assert status==200
    artifacts=json.loads(body)['artifacts']
    assert len(artifacts)==3
    for artifact in artifacts:
        status,_,payload=http_request(console,path=artifact['url'])
        assert status==200 and payload
    assert not console.runtime.calls and console.job is None
    costs['fresh']=False
    assert http_request(console,method='POST',path='/api/map/export',body=b'{}')[0]==409
    assert len(list((console.runtime.session_dir/'map_snapshots').iterdir()))==1


def test_shared_view_export_uses_owned_tasks_and_rejects_missing_sources(console,monkeypatch):
    grid=dict(available=True,frame_id='map',width=2,height=2,resolution=1.,
              origin=dict(x=0.,y=0.,yaw=0.),data=[0,0,0,0])
    monkeypatch.setattr(console.monitor,'map_snapshot',lambda:dict(grid))
    status,_,body=http_request(console,method='POST',path='/api/map/export',body=b'{"view":"rooms"}')
    assert status==200
    for artifact in json.loads(body)['artifacts']:
        assert http_request(console,path=artifact['url'])[0]==200
    before=list((console.runtime.session_dir/'map_snapshots').iterdir())
    for request,expected in [(b'{"view":"../../secret"}',400),(b'{"view":[]}',400),
                             (b'{"view":"rooms","file":"secret"}',400),
                             (b'{"view":"semantic","room":[]}',400),
                             (b'{"view":"observations","task_id":"../../old"}',404),
                             (b'{"view":"observations"}',409),(b'{"view":"clean_coverage"}',409)]:
        assert http_request(console,method='POST',path='/api/map/export',body=request)[0]==expected
    assert list((console.runtime.session_dir/'map_snapshots').iterdir())==before
    assert not console.runtime.calls and console.job is None


def test_clean_export_reads_only_owned_persisted_brush_grid_and_retains_partial(console,monkeypatch,tmp_path):
    import time
    import numpy as np
    console.rooms={'utility':dict(bounds=[0,0,2,2],aliases=['杂物间'],display_name='杂物间')}
    grid=dict(available=True,frame_id='map',width=2,height=2,resolution=1.,origin=dict(x=0.,y=0.,yaw=0.),data=[0]*4)
    monkeypatch.setattr(console.monitor,'map_snapshot',lambda:dict(grid))
    source=console.runtime.session_dir/'requests/clean_saved';source.mkdir(parents=True)
    result=dict(frame_id='map',room='utility',status='partial',coverage=dict(floor_cells=4,eligible_cells=4,covered_cells=1,coverage_fraction=.25))
    documents={'clean_result.json':result,'clean_plan.json':dict(frame_id='map',origin=[0.,0.,0.],resolution_m=1.),'clean_trace.json':dict(frame_id='map',samples=[])}
    for name,value in documents.items():(source/name).write_text(json.dumps(value))
    np.savez(source/'clean_navigation.npz',static_map=np.zeros((2,2),dtype=np.int8),eligible=np.ones((2,2),dtype=bool),covered_001=np.array([[True,False],[False,False]]))
    raw=(source/'clean_navigation.npz').read_bytes()
    row=dict(id='clean_saved',kind='clean',state='partial',result=result,_folder=source,_started=time.monotonic(),_ended=time.monotonic())
    console.jobs[row['id']]=row
    status,_,body=http_request(console,method='POST',path='/api/map/export',body=b'{"view":"clean_coverage","task_id":"clean_saved"}')
    assert status==200
    data=json.loads(http_request(console,path=json.loads(body)['artifacts'][-1]['url'])[2])
    assert data['cleaning']['result']['status']=='partial'
    assert data['cleaning']['result']['coverage']['covered_cells']==1
    assert sum(sum(v)for v in data['cleaning']['covered'])==1
    assert (source/'clean_navigation.npz').read_bytes()==raw
    row['_folder']=tmp_path/'outside_session'
    assert http_request(console,method='POST',path='/api/map/export',body=b'{"view":"clean_coverage","task_id":"clean_saved"}')[0]==409
    assert not console.runtime.calls


@pytest.mark.parametrize('seconds,timeout',[(0,720),(180,900)])
def test_room_supplement_budget_survives_parent_watchdog(console,seconds,timeout):
    console.rooms['gym']['cleaning_supplement_seconds']=seconds
    console.submit(dict(request_id='supplement_budget_01',mode='clean',room='gym'))
    argv=console.runtime.calls[-1][1]
    assert float(argv[argv.index('--timeout')+1])==timeout
    assert ('--supplement-seconds' in argv)==bool(seconds)
    limits=[]
    console._wait_client=lambda row,process,limit,started:limits.append(limit)
    folder=console.job['_folder']
    put_json(folder/'clean_result.json',dict(status='partial',navigation_settled=True))
    put_json(folder/'clean_trace.json',[])
    console.runtime.calls[-1][2].returncode=0
    console.pending_workers[-1].run()
    assert limits==[timeout+12]
    assert console.snapshot()['task']['state']=='partial'


@pytest.mark.parametrize('seconds',[True,-1,181,float('nan'),'180'])
def test_invalid_supplement_budget_never_starts_motion(console,seconds):
    console.rooms['gym']['cleaning_supplement_seconds']=seconds
    assert_api_error(500,lambda:console.submit(dict(request_id='supplement_bad_01',mode='clean',room='gym')))
    assert not console.runtime.calls
    assert console.snapshot()['task']['state']=='failed'
