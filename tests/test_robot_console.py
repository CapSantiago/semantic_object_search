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


def finish_search(console, status='found', code=0, task_id='task_001'):
    folder = console.job['_folder']
    result = dict(status=status, count=1 if status == 'found' else 0, objects=[])
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


def assert_api_error(status, action):
    with pytest.raises(module.ApiError) as caught:
        action()
    assert caught.value.status == status


def test_command_normalization_uses_existing_room_and_object_parser(console):
    _, _, structured = console.normalize(task())
    request_id, original, natural = console.normalize(
        {'request_id': 'request_001', 'command': '  帮我去厨房找杯子  '})
    assert request_id == 'request_001'
    assert original == '帮我去厨房找杯子'
    assert json.loads(structured) == dict(mode='find', room='kitchen', category='cup')
    assert natural == structured
    assert json.loads(console.normalize(task(mode='count'))[2])['mode'] == 'count'


def test_progress_is_visible_only_for_the_matching_request(console):
    first = console.submit(task())['task']
    event = dict(request_id=first['id'], task_id='search_001', stage='observing', count=1)
    console.monitor.values['search_progress'] = event
    assert console.snapshot()['task']['progress'] == event
    finish_search(console)
    next_task = console.submit(task('request_002'))['task']
    assert 'progress' not in next_task


def test_cleaning_shares_request_lock_and_persists_its_own_results(console):
    request = dict(request_id='clean_room_01', mode='clean', room='gym')
    submitted = console.submit(request)
    assert submitted['busy'] and submitted['task']['kind']=='clean'
    assert '/run_cleaning.py' in console.runtime.calls[-1][1][1]
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
