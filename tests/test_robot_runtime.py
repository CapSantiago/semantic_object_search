"""Real local subprocess checks for ownership and cleanup; no ROS stack starts."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('robot_runtime', ROOT/'scripts/robot_runtime.py')
runtime_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime_module)
Runtime = runtime_module.Runtime


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.runtimes = []
        self.addCleanup(self.close_runtimes)

    def close_runtimes(self):
        for runtime in reversed(self.runtimes):
            runtime.close(graceful_seconds=.15)

    def make_runtime(self, **kwargs):
        options = dict(domain=71, headless=True, check_dependencies=False)
        options.update(kwargs)
        runtime = Runtime(self.root, **options)
        self.runtimes.append(runtime)
        return runtime

    def wait_file(self, path, timeout=3):
        deadline = time.monotonic()+timeout
        while not path.exists() and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(path.exists(), str(path))

    def test_duplicate_owner_is_rejected_and_close_releases_locks(self):
        first = self.make_runtime()
        with self.assertRaisesRegex(RuntimeError, '已有一键运行实例'):
            self.make_runtime()
        first.close(graceful_seconds=0)
        second = self.make_runtime()
        self.assertNotEqual(first.session_dir, second.session_dir)
        self.assertNotEqual(first.env['IGN_PARTITION'], second.env['IGN_PARTITION'])
        self.assertEqual(second.env['ROS_DOMAIN_ID'], '71')
        self.assertEqual(second.snapshot()['simulation']['state'], 'not_started')
        self.assertEqual(second.snapshot()['rviz']['state'], 'disabled')

    def test_depth_projection_source_is_explicit_and_forwarded_to_simulation(self):
        native=self.make_runtime()
        self.assertEqual(native.config['depth_cloud_source'],'native')
        self.assertIn('depth_cloud_source:=native',native.command('simulation'))
        native.close(graceful_seconds=0)
        image=self.make_runtime(depth_cloud_source='image')
        self.assertEqual(image.config['depth_cloud_source'],'image')
        self.assertIn('depth_cloud_source:=image',image.command('simulation'))

    def test_sim_localization_candidate_is_explicit_and_separately_recorded(self):
        base=self.make_runtime(precision_docking=True)
        self.assertFalse(base.config['sim_localization_candidate'])
        self.assertEqual(base.config['navigation'],'src/semantic_search/config/nav2_depth_docking_progress.yaml')
        base.close(graceful_seconds=0)
        candidate=self.make_runtime(precision_docking=True,sim_localization_candidate=True)
        self.assertTrue(candidate.config['sim_localization_candidate'])
        self.assertEqual(candidate.config['navigation'],'src/semantic_search/config/nav2_depth_sim_localization.yaml')

    def test_sim_localization_candidate_rejects_unsupported_combinations_before_ownership(self):
        for extra in ({},{'precision_docking':True,'body_footprint':True},
                      {'precision_docking':True,'navigation_safety':True}):
            with self.assertRaisesRegex(ValueError,'仿真定位候选'):
                self.make_runtime(sim_localization_candidate=True,**extra)
        self.make_runtime()

    def test_unknown_depth_projection_source_is_rejected_before_locking(self):
        with self.assertRaisesRegex(ValueError,'点云来源'):
            self.make_runtime(depth_cloud_source='unverified')
        self.make_runtime()

    def test_navigation_map_selection_is_recorded_and_frozen_with_its_image(self):
        from semantic_search.map_resources import resolve_map
        config=self.root/'src/semantic_search/config';config.mkdir(parents=True)
        (config/'map_resources.json').write_text(json.dumps({'version':1,'maps':{
            'aws_house_v1':{'yaml':'maps/house.yaml','compatible_yaml':[]}}}))
        folder=self.root/'maps';folder.mkdir()
        (folder/'house.yaml').write_text('image: house.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
        (folder/'house.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
        def dependencies(runtime):runtime.map_resource=resolve_map(runtime.root,map_id='aws_house_v1')
        with patch.object(Runtime,'_check_dependencies',dependencies):runtime=self.make_runtime(check_dependencies=True)
        self.assertEqual(runtime.config['map'],'maps/house.yaml')
        self.assertIn('map:='+str(folder/'house.yaml'),runtime.command('navigation'))
        metadata=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertEqual(metadata['config']['map_resource']['image_sha256'],runtime.map_resource['image_sha256'])
        frozen=runtime.session_dir/metadata['configuration_snapshots']['maps/house.pgm']
        original=frozen.read_bytes();(folder/'house.pgm').write_bytes(b'new-version')
        self.assertEqual(frozen.read_bytes(),original)

    def test_explicit_candidate_categories_reach_the_search_process_and_session_record(self):
        runtime = self.make_runtime(candidate_categories=['bowl','cell phone','bowl'])
        self.assertEqual(runtime.config['candidate_categories'],['bowl','cell phone'])
        self.assertEqual(runtime.config['target_categories'],['cup','bottle','chair','bowl','cell phone'])
        command = runtime.command('search')
        parameter = next(item for item in command if item.startswith('target_categories:='))
        self.assertEqual(json.loads(parameter.split(':=',1)[1]), runtime.config['target_categories'])
        recorded = json.loads((runtime.session_dir/'session.json').read_text())
        self.assertEqual(recorded['config']['candidate_categories'],['bowl','cell phone'])
        runtime.close(graceful_seconds=0)
        with self.assertRaises(ValueError):
            self.make_runtime(candidate_categories=['dumbbell'])

    def test_candidate_camera_is_recorded_and_passed_to_simulation_and_tf_builder(self):
        runtime=self.make_runtime(camera_height=1.2,camera_pitch=.4,
                                  camera_resolution='1280x960',camera_fov=1.4)
        self.assertIn('camera_height:=1.2',runtime.command('simulation'))
        self.assertIn('camera_pitch:=0.4',runtime.command('simulation'))
        self.assertEqual(runtime.config['camera_height'],1.2)
        self.assertIn('image_width:=1280',runtime.command('simulation'))
        self.assertIn('image_height:=960',runtime.command('simulation'))
        self.assertIn('camera_hfov:=1.4',runtime.command('simulation'))
        runtime.close(graceful_seconds=0)
        for options in (dict(camera_height=float('nan')),dict(camera_height=2),dict(camera_pitch=-.1),
                        dict(camera_resolution='4k'),dict(camera_fov=float('nan'))):
            with self.assertRaises(ValueError):
                self.make_runtime(**options)

    def test_local_cleaning_recovery_is_explicitly_recorded_and_defaults_off(self):
        for enabled in (False, True):
            runtime = self.make_runtime(cleaning_local_recovery=enabled)
            self.assertEqual(runtime.config['cleaning_local_recovery'], enabled)
            metadata = json.loads((runtime.session_dir/'session.json').read_text())
            self.assertEqual(metadata['config']['cleaning_local_recovery'], enabled)
            runtime.close(graceful_seconds=0)

    def test_budget_cleaning_selection_is_recorded_and_defaults_off(self):
        for options, expected in (({},False),({'cleaning_budget_selection':True},True)):
            runtime=self.make_runtime(**options)
            self.assertEqual(runtime.config['cleaning_budget_selection'],expected)
            metadata=json.loads((runtime.session_dir/'session.json').read_text())
            self.assertEqual(metadata['config']['cleaning_budget_selection'],expected)
            runtime.close(graceful_seconds=0)

    def test_narrow_transit_rooms_cannot_start_with_the_circle_profile(self):
        source=self.root/'rooms.json'
        source.write_bytes((ROOT/'src/semantic_search/config/rooms_aws_house_v12_body_transit.json').read_bytes())
        with self.assertRaisesRegex(ValueError,'必须同时显式启用'):
            self.make_runtime(rooms_file='rooms.json')
        runtime=self.make_runtime(rooms_file='rooms.json',body_footprint=True)
        self.assertEqual(runtime.config['rooms'],'rooms.json')
        metadata=json.loads((runtime.session_dir/'session.json').read_text())
        frozen=runtime.session_dir/metadata['configuration_snapshots']['rooms.json']
        self.assertEqual(frozen.read_bytes(),source.read_bytes())

    def test_body_footprint_is_explicit_recorded_and_not_combined_with_safety(self):
        baseline=self.make_runtime()
        self.assertFalse(baseline.config['body_footprint'])
        baseline.close(graceful_seconds=0)
        candidate=self.make_runtime(body_footprint=True)
        self.assertTrue(candidate.config['body_footprint'])
        self.assertEqual(candidate.config['navigation'],'src/semantic_search/config/nav2_body_footprint.yaml')
        self.assertIn('params_file:='+str(self.root/candidate.config['navigation']),candidate.command('navigation'))
        candidate.close(graceful_seconds=0)
        with self.assertRaisesRegex(ValueError,'分别启用'):
            self.make_runtime(body_footprint=True,navigation_safety=True)

    def test_navigation_safety_selects_the_protected_simulator_input_and_costmaps(self):
        runtime=self.make_runtime(navigation_safety=True)
        self.assertIn('velocity_topic:=/cmd_vel_safe',runtime.command('simulation'))
        self.assertIn('navigation_safety.launch.py',runtime.command('safety'))
        self.assertTrue(runtime.config['navigation_safety'])
        self.assertEqual(runtime.config['navigation'],'src/semantic_search/config/nav2_safety.yaml')
        self.assertIn('params_file:='+str(self.root/'src/semantic_search/config/nav2_safety.yaml'),
                      runtime.command('navigation'))
        self.assertEqual(runtime.snapshot()['safety']['state'],'not_started')

    def test_charging_config_is_map_bound_frozen_and_explicitly_passed(self):
        import hashlib
        station=json.loads((ROOT/'src/semantic_search/config/charging_station.json').read_text())
        folder=self.root/'maps';folder.mkdir()
        (folder/'house.yaml').write_text('image: house.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        (folder/'house.pgm').write_bytes(b'isolated-map')
        station['map_yaml']='maps/house.yaml'
        station['map_sha256']={key:hashlib.sha256((folder/name).read_bytes()).hexdigest()
            for key,name in [('yaml','house.yaml'),('image','house.pgm')]}
        source=self.root/'src/semantic_search/config/charging_station.json';source.parent.mkdir(parents=True)
        source.write_text(json.dumps(station));runtime=self.make_runtime()
        self.assertTrue(runtime.config['charging_station_enabled'])
        frozen=runtime.charging_station_file
        self.assertEqual(json.loads(frozen.read_text()),station)
        self.assertEqual(runtime.env['SEMANTIC_CHARGING_CONFIG'],str(frozen))
        self.assertIn('charging_file:='+str(frozen),runtime.command('simulation'))
        station['map_pose']=[99.,99.,0.];source.write_text(json.dumps(station))
        self.assertEqual(runtime.charging_station['map_pose'],[-1.8,-4.,0.])
        runtime.close(graceful_seconds=0)
        disabled=self.make_runtime(charging_station=False,env={'SEMANTIC_CHARGING_CONFIG':'foreign'})
        self.assertIsNone(disabled.charging_station)
        self.assertNotIn('SEMANTIC_CHARGING_CONFIG',disabled.env)

    def test_candidate_model_and_input_size_are_explicit_and_fingerprinted(self):
        (self.root/'yolov8s-seg.pt').write_bytes(b'isolated-test-model')
        runtime=self.make_runtime(model='yolov8s-seg.pt',image_size=1280)
        self.assertIn('model:='+str(self.root/'yolov8s-seg.pt'),runtime.command('detector'))
        self.assertIn('image_size:=1280',runtime.command('detector'))
        stored=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertIn('yolov8s-seg.pt',stored['source_sha256'])
        self.assertEqual(stored['config']['detector']['image_size'],1280)

    def test_ball_auxiliary_is_explicit_passed_and_frozen_without_replacing_primary(self):
        self.assertFalse(any('ball_model:=' in p for p in self.make_runtime().command('detector')))
        self.runtimes[-1].close(graceful_seconds=0)
        name='yoloe-11s-seg-pf.pt'
        (self.root/name).write_bytes(b'isolated-test-auxiliary')
        runtime=self.make_runtime(ball_model=name)
        command=runtime.command('detector')
        self.assertIn('ball_model:='+str(self.root/name),command)
        self.assertIn('model:='+str(self.root/'yolov8n-seg.pt'),command)
        stored=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertIn(name,stored['source_sha256'])
        self.assertEqual(stored['config']['detector']['ball_max_rate_hz'],2.)
        with self.assertRaises(ValueError):self.make_runtime(ball_model='unlocked.pt')

    def test_prompted_auxiliary_freezes_embeddings_and_keeps_primary_model(self):
        name='yoloe-11s-seg.pt';(self.root/name).write_bytes(b'isolated-prompted-model')
        resource=self.root/'resources';resource.mkdir()
        (resource/'ball_prompts_v1.npz').write_bytes(b'isolated-embeddings')
        (resource/'vision_prompts_v1.json').write_text('{}')
        runtime=self.make_runtime(auxiliary_model=name,auxiliary_categories=['book','sports ball'])
        command=runtime.command('detector')
        self.assertIn('model:='+str(self.root/'yolov8n-seg.pt'),command)
        self.assertIn('auxiliary_model:='+str(self.root/name),command)
        self.assertIn('auxiliary_prompts:='+str(resource/'ball_prompts_v1.npz'),command)
        stored=json.loads((runtime.session_dir/'session.json').read_text())
        key='resources/ball_prompts_v1.npz'
        self.assertIn(key,stored['configuration_snapshots'])
        self.assertEqual((runtime.session_dir/stored['configuration_snapshots'][key]).read_bytes(),b'isolated-embeddings')
        with self.assertRaises(ValueError):self.make_runtime(ball_model='yoloe-11s-seg-pf.pt',auxiliary_model=name)
        with self.assertRaises(ValueError):self.make_runtime(auxiliary_model=name,auxiliary_categories=['apple'])

    def test_dependency_and_display_failure_release_locks(self):
        with self.assertRaisesRegex(RuntimeError, '运行资源缺失'):
            self.make_runtime(check_dependencies=True)
        self.assertFalse((self.root/'results/sessions').exists())
        with self.assertRaisesRegex(RuntimeError, '--headless'):
            self.make_runtime(headless=False, env={})
        runtime = self.make_runtime()
        self.assertTrue(runtime.session_dir.is_dir())

    def test_session_retains_the_actual_navigation_config_after_later_edits(self):
        source = self.root/'src/semantic_search/config/nav2_safety.yaml'
        source.parent.mkdir(parents=True)
        source.write_text('robot_radius: 0.25\n')
        runtime = self.make_runtime(navigation_safety=True)
        stored = json.loads((runtime.session_dir/'session.json').read_text())
        name = str(source.relative_to(self.root))
        snapshot = runtime.session_dir/stored['configuration_snapshots'][name]
        source.write_text('robot_radius: 0.30\n')
        self.assertEqual(snapshot.read_text(),'robot_radius: 0.25\n')
        import hashlib
        self.assertEqual(hashlib.sha256(snapshot.read_bytes()).hexdigest(),stored['source_sha256'][name])

    def test_different_projects_cannot_own_the_same_ros_domain(self):
        first = self.make_runtime()
        second_root = self.root/'second_project'
        second_root.mkdir()
        with self.assertRaisesRegex(RuntimeError, '已有一键运行实例'):
            Runtime(second_root, domain=71, headless=True, check_dependencies=False)
        # The failed second domain lock must also release its project lock.
        second = Runtime(second_root, domain=72, headless=True, check_dependencies=False)
        self.runtimes.append(second)
        self.assertTrue(first.session_dir.is_dir())
        self.assertEqual(second.domain, 72)

    def test_existing_session_is_not_modified(self):
        existing = self.root/'already_saved'
        existing.mkdir()
        (existing/'result.json').write_text('keep original')
        with self.assertRaises(FileExistsError):
            self.make_runtime(session_dir=existing)
        self.assertEqual(sorted(p.name for p in existing.iterdir()), ['result.json'])
        self.assertEqual((existing/'result.json').read_text(), 'keep original')
        runtime = self.make_runtime()
        self.assertNotEqual(runtime.session_dir, existing)

    def test_explicit_fixture_is_used_and_fingerprinted(self):
        rooms=self.root/'rooms.json'
        rooms.write_text(json.dumps({'test':dict(aliases=['测试'],bounds=[0,0,2,2],views=[[1,1,0]])}))
        layout=self.root/'layout.json'
        layout.write_text(json.dumps(dict(targets=[])))
        runtime=self.make_runtime(rooms_file='rooms.json',layout_file='layout.json')
        self.assertIn('rooms_file:='+str(rooms),runtime.command('search'))
        self.assertIn('layout_file:=layout.json',runtime.command('simulation'))
        receipt=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertEqual(receipt['config']['rooms'],'rooms.json')
        self.assertIn('rooms.json',receipt['source_sha256'])

    def test_invalid_fixture_releases_runtime_locks(self):
        with self.assertRaises(ValueError):
            self.make_runtime(rooms_file='../missing.json')
        runtime=self.make_runtime()
        self.assertEqual(runtime.config['rooms'],'src/semantic_search/config/rooms_aws_house_v6.json')

    def test_current_candidate_arguments_are_explicit(self):
        runtime = self.make_runtime()
        simulation = runtime.command('simulation')
        self.assertIn('gui:=false', simulation)
        self.assertIn('layout_file:=scenes/aws_house/layout_service_surfaces_v2.json', simulation)
        self.assertIn('rooms_file:='+str(self.root/'src/semantic_search/config/rooms_aws_house_v6.json'),
                      runtime.command('search'))
        for component in ('search','detector'):
            self.assertIn('sensor_reliability:=reliable', runtime.command(component))
        self.assertIn('device:=cuda:0',runtime.command('detector'))
        self.assertEqual(runtime.env['SEMANTIC_DETECTOR_DEVICE'],'cuda:0')
        self.assertIn('output_dir:='+str(runtime.task_output_dir), runtime.command('search'))
        with self.assertRaises(ValueError):
            runtime.command('rviz')
        with self.assertRaises(ValueError):
            runtime.start_process('../escape', [sys.executable, '-c', 'pass'])

    def test_cpu_fallback_is_explicit_and_recorded(self):
        runtime=self.make_runtime(device='cpu')
        self.assertIn('device:=cpu',runtime.command('detector'))
        self.assertEqual(runtime.env['SEMANTIC_DETECTOR_DEVICE'],'cpu')
        receipt=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertEqual(receipt['config']['detector']['python_env'],'.venv')
        self.assertFalse(receipt['config']['detector']['half'])

    def test_invalid_device_cannot_acquire_locks(self):
        with self.assertRaises(ValueError):self.make_runtime(device='invalid')
        self.make_runtime(device='cpu')

    def test_logs_exit_codes_and_failed_exec_are_preserved(self):
        runtime = self.make_runtime()
        process = runtime.start_process('success', [sys.executable, '-c', 'print("actual child output",flush=True)'])
        self.assertEqual(process.wait(timeout=3), 0)
        self.assertIn('actual child output', runtime.log_tail('success'))
        self.assertEqual(runtime.snapshot()['success']['returncode'], 0)
        with self.assertRaises(RuntimeError):
            runtime.start_process('success', [sys.executable, '-c', 'pass'])
        with self.assertRaises(FileNotFoundError):
            runtime.start_process('missing', ['/definitely/missing/robot/executable'])
        receipt = runtime.close(graceful_seconds=.1)
        self.assertEqual(receipt['status'], 'stopped')
        self.assertEqual(runtime.close(), receipt)
        self.assertTrue((runtime.session_dir/'shutdown.json').is_file())

    def test_signal_only_targets_named_owned_client(self):
        runtime = self.make_runtime()
        ready = runtime.session_dir/'ready'
        code = ('import signal,time;from pathlib import Path;'
                'signal.signal(signal.SIGINT,lambda *args:exit(23));'
                f'Path({str(ready)!r}).write_text("ready");'
                'time.sleep(60)')
        client = runtime.start_process('task_client', [sys.executable, '-c', code])
        other = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'])
        try:
            self.wait_file(ready)
            self.assertTrue(runtime.signal_process('task_client'))
            self.assertEqual(client.wait(timeout=3), 23)
            self.assertIsNone(other.poll())
            runtime.close(graceful_seconds=.1)
            self.assertIsNone(other.poll(), 'cleanup touched an unrelated child')
        finally:
            other.terminate()
            other.wait(timeout=3)

    def test_detached_descendants_are_cleaned_after_parent_exits(self):
        runtime = self.make_runtime()
        child_pid = runtime.session_dir/'detached.pid'
        child_code = 'import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);time.sleep(60)'
        parent_code = ('import subprocess,sys;from pathlib import Path;'
                       f'child=subprocess.Popen([sys.executable,"-c",{child_code!r}],start_new_session=True);'
                       f'Path({str(child_pid)!r}).write_text(str(child.pid))')
        parent = runtime.start_process('detaching_parent', [sys.executable, '-c', parent_code])
        parent.wait(timeout=3)
        self.wait_file(child_pid)
        pid = int(child_pid.read_text())
        self.assertIsNotNone(runtime_module._identity(pid, runtime.marker))
        receipt = runtime.close(graceful_seconds=.1)
        self.assertEqual(receipt['status'], 'stopped')
        self.assertIsNone(runtime_module._identity(pid, runtime.marker))
        self.assertTrue(any(row['pid']==pid and row['signal']=='SIGTERM' for row in receipt['signals']))

    def test_pid_identity_mismatch_is_never_signalled(self):
        runtime = self.make_runtime()
        process = runtime.start_process('sleeper', [sys.executable, '-c', 'import time;time.sleep(60)'])
        identity = runtime_module._identity(process.pid, runtime.marker)
        self.assertIsNotNone(identity)
        wrong = dict(identity, start_ticks=identity['start_ticks']+1)
        self.assertFalse(runtime_module._signal(wrong, runtime.marker, signal.SIGTERM))
        self.assertIsNone(process.poll())

    def test_registration_write_failure_reaps_owned_processes_and_releases_locks(self):
        runtime = self.make_runtime()
        ready = runtime.session_dir / 'registered_child.pid'
        parent_code = (
            'import subprocess,sys,time;from pathlib import Path;'
            'child=subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"],'
            'start_new_session=True);'
            f'Path({str(ready)!r}).write_text(str(child.pid));time.sleep(60)')

        def failed_write():
            self.wait_file(ready)
            raise OSError('simulated registration write failure')

        runtime._write_processes = failed_write
        with self.assertRaisesRegex(OSError, 'simulated registration write failure'):
            runtime.start_process('unpersisted_client', [sys.executable, '-c', parent_code])
        row = runtime.snapshot()['unpersisted_client']
        self.assertEqual(row['state'], 'exited')
        self.assertIsNone(runtime_module._identity(row['pid'], runtime.marker))
        self.assertIsNone(runtime_module._identity(int(ready.read_text()), runtime.marker))
        self.assertEqual(json.loads((runtime.session_dir / 'shutdown.json').read_text())['status'], 'stopped')
        with self.assertRaisesRegex(RuntimeError, 'already closed'):
            runtime.start_process('retry', [sys.executable, '-c', 'pass'])
        replacement = self.make_runtime()
        self.assertNotEqual(replacement.session_dir, runtime.session_dir)

    def test_transient_identity_lookup_does_not_disable_cancellation(self):
        runtime = self.make_runtime()
        original_identity = runtime_module._identity
        lookups = []

        def delayed_identity(pid, marker):
            lookups.append(pid)
            return None if len(lookups) == 1 else original_identity(pid, marker)

        ready = runtime.session_dir / 'cancel_ready'
        code = ('import signal,time;from pathlib import Path;'
                'signal.signal(signal.SIGINT,lambda *args:exit(23));'
                f'Path({str(ready)!r}).write_text("ready");time.sleep(60)')
        with patch.object(runtime_module, '_identity', side_effect=delayed_identity):
            child = runtime.start_process('cancel_client', [sys.executable, '-c', code])
        self.assertGreaterEqual(len(lookups), 2)
        self.wait_file(ready)
        # Also exercise recovery after an unusually long initial /proc delay.
        runtime._processes['cancel_client']['identity'] = None
        self.assertTrue(runtime.signal_process('cancel_client', signal.SIGINT))
        self.assertEqual(child.wait(timeout=3), 23)


if __name__ == '__main__':
    unittest.main()
