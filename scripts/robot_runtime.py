#!/usr/bin/env python3
"""Own and supervise one local robot session; ROS readiness is checked elsewhere.

The supervisor never sends a system-wide kill or trusts a stale PID. Descendants
inherit a unique ownership marker, including Nav2's separate setsid session, and
are signalled through pidfds after UID, marker and process-start verification.
"""
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from uuid import uuid4


COMPONENTS = ('simulation', 'navigation', 'detector', 'search', 'rviz')
OWNER_VARIABLE = 'SEMANTIC_RUNTIME_OWNER'


def _save(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def _identity(pid, marker):
    """Return a matching live process identity, without exposing its environment."""
    path = Path('/proc') / str(pid)
    try:
        if path.stat().st_uid != os.getuid():
            return None
        if marker not in (path / 'environ').read_bytes().split(b'\0'):
            return None
        # Everything after the final ')' begins at /proc stat field 3.
        fields = (path / 'stat').read_text().rpartition(')')[2].split()
        if fields[0] == 'Z':
            return None
        return dict(pid=int(pid), start_ticks=int(fields[19]), state=fields[0])
    except (OSError, ValueError, IndexError):
        return None


def _owned(marker):
    rows = []
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            row = _identity(int(path.name), marker)
            if row is not None:
                rows.append(row)
    return rows


def _signal(identity, marker, sig):
    """Pin the process first, then verify its identity before sending a signal."""
    try:
        descriptor = os.pidfd_open(identity['pid'])
    except (ProcessLookupError, PermissionError):
        return False
    try:
        current = _identity(identity['pid'], marker)
        if current is None or current['start_ticks'] != identity['start_ticks']:
            return False
        signal.pidfd_send_signal(descriptor, sig)
        return True
    except (ProcessLookupError, PermissionError):
        return False
    finally:
        os.close(descriptor)


class Runtime:
    """Pure-Python lifecycle owner used by robot_console.py.

    ``check_dependencies=False`` is for isolated subprocess tests. It does not
    bypass locks or ownership checks. It must not be exposed as a UI control.
    """

    def __init__(self, root, domain=66, headless=False, session_dir=None, env=None,
                 check_dependencies=True):
        if type(domain) is not int or not 0 <= domain <= 232:
            raise ValueError('ROS domain must be an integer from 0 to 232')
        if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
            raise RuntimeError('Linux pidfd support is required for safe process cleanup')
        self.root = Path(root).resolve()
        self.domain = domain
        self.headless = bool(headless)
        self.env = dict(os.environ if env is None else env)
        self.token = uuid4().hex
        self.marker = (OWNER_VARIABLE + '=' + self.token).encode()
        self._locks = []
        self._processes = {}
        self._mutex = threading.RLock()
        self._closed = False
        self._receipt = None
        self.session_dir = None
        created_session = False
        try:
            self._acquire_lock(self.root / 'results' / '.robot-runtime.lock')
            self._acquire_lock(Path('/tmp') / f'semantic_robot_{os.getuid()}_domain_{domain}.lock')
            if not self.headless and not self.env.get('DISPLAY'):
                raise RuntimeError('没有可用 DISPLAY；在桌面终端运行，或使用 --headless 关闭 Gazebo/RViz 窗口')
            if check_dependencies:
                self._check_dependencies()
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            self.session_dir = (Path(session_dir).resolve() if session_dir is not None
                                else self.root / 'results' / 'sessions' / (stamp + '_' + self.token[:8]))
            self.session_dir.mkdir(parents=True, exist_ok=False)
            created_session = True
            self.logs_dir = self.session_dir / 'logs'
            self.logs_dir.mkdir()
            self.task_output_dir = self.session_dir / 'tasks'
            self.task_output_dir.mkdir()
            self.env.update(SEMANTIC_ROS_DOMAIN_ID=str(domain), ROS_DOMAIN_ID=str(domain),
                            ROS_LOCALHOST_ONLY='1', SEMANTIC_USE_SYSTEM_DDS='0',
                            IGN_PARTITION='semantic_session_' + self.token,
                            PYTHONUNBUFFERED='1')
            self.env[OWNER_VARIABLE] = self.token
            # env.sh preserves these explicit selectors. Do not inherit a
            # workstation's remote discovery configuration into the console.
            for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                        'FASTDDS_DEFAULT_PROFILES_FILE'):
                self.env.pop(key, None)
            self.config = dict(domain=domain, ign_partition=self.env['IGN_PARTITION'],
                               headless=self.headless, scene_id='aws_house_console_' + self.token[:8],
                               layout='scenes/aws_house/layout_r0_yellow_v1.json',
                               rooms='src/semantic_search/config/rooms_aws_house_v4.json',
                               navigation='src/semantic_search/config/nav2_depth.yaml',
                               sensor_reliability='reliable', model='yolov8n-seg.pt',
                               camera_height=1.0, camera_pitch=0.25,
                               detector=dict(device='cpu', image_size=640, confidence=.35, iou=.30))
            fingerprints = {}
            for name in (self.config['layout'], self.config['rooms'], self.config['navigation'],
                         self.config['model'], 'src/semantic_search/semantic_search/search_node.py',
                         'src/semantic_search/semantic_search/detector.py',
                         'src/semantic_search/semantic_search/sensor_qos.py',
                         'scripts/start_robot.sh', 'scripts/robot_console.py',
                         'scripts/robot_runtime.py', 'scripts/robot_monitor.py',
                         'ui/index.html', 'ui/app.js', 'ui/style.css'):
                path = self.root / name
                if path.is_file():
                    fingerprints[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            _save(self.session_dir / 'session.json', dict(
                started_utc=datetime.now(timezone.utc).isoformat(), pid=os.getpid(),
                root=str(self.root), session_dir=str(self.session_dir), config=self.config,
                source_sha256=fingerprints,
                note='Process running is not a sensor/localization/navigation readiness claim.'))
        except BaseException as error:
            try:
                if created_session:
                    _save(self.session_dir / 'startup_failure.json', dict(
                        error=f'{type(error).__name__}: {error}',
                        at_utc=datetime.now(timezone.utc).isoformat()))
            finally:
                self._release_locks()
            raise

    def _check_dependencies(self):
        required = ('scripts/development.sh', 'scripts/development_navigation.sh',
                    'scripts/development_detector.sh', 'scripts/development_search.sh',
                    'scripts/rviz.sh', 'install/setup.bash', '.venv/bin/python', 'yolov8n-seg.pt',
                    'scenes/aws_house/layout_r0_yellow_v1.json',
                    'src/semantic_search/config/rooms_aws_house_v4.json',
                    'src/semantic_search/config/nav2_depth.yaml', 'scenes/aws_house/r0_v1/maps/house.yaml')
        missing = [name for name in required if not (self.root / name).is_file()]
        if missing:
            raise RuntimeError('运行资源缺失：' + ', '.join(missing)
                               + '。请按部署说明准备资源，并执行 bash scripts/build.sh')

    def _acquire_lock(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(descriptor)
            raise RuntimeError(f'已有一键运行实例占用项目或 ROS 域 {self.domain}，请先退出已有实例') from None
        self._locks.append(descriptor)
        os.ftruncate(descriptor, 0)
        os.write(descriptor, json.dumps(dict(pid=os.getpid(), root=str(self.root), domain=self.domain)).encode())

    def _release_locks(self):
        for descriptor in reversed(self._locks):
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)
        self._locks.clear()

    def command(self, name):
        """Return argv only. No user text is ever interpolated into a shell."""
        paths = self.config
        if name == 'simulation':
            return ['bash', str(self.root / 'scripts/development.sh'),
                    'gui:=' + ('false' if self.headless else 'true'),
                    'layout_file:=' + paths['layout'], 'evidence_dir:=' + str(self.session_dir / 'scene')]
        if name == 'navigation':
            return ['bash', str(self.root / 'scripts/development_navigation.sh')]
        if name == 'detector':
            return ['bash', str(self.root / 'scripts/development_detector.sh'),
                    '-p', 'sensor_reliability:=reliable']
        if name == 'search':
            return ['bash', str(self.root / 'scripts/development_search.sh'),
                    '-p', 'rooms_file:=' + str(self.root / paths['rooms']),
                    '-p', 'scene_id:=' + paths['scene_id'],
                    '-p', 'output_dir:=' + str(self.task_output_dir),
                    '-p', 'sensor_reliability:=reliable']
        if name == 'rviz':
            if self.headless:
                raise ValueError('RViz is disabled in --headless mode')
            return ['bash', str(self.root / 'scripts/rviz.sh'), 'scene:=aws_house']
        raise ValueError('Unknown robot component: ' + str(name))

    def start_component(self, name):
        return self.start_process(name, self.command(name))

    def start_process(self, name, argv):
        """Register a trusted argv child, including a task client or checker."""
        if not re.fullmatch(r'[a-z][a-z0-9_-]{0,95}', name):
            raise ValueError('Invalid process name')
        if not isinstance(argv, (list, tuple)) or not argv or not all(isinstance(x, str) for x in argv):
            raise ValueError('argv must be a nonempty sequence of strings')
        with self._mutex:
            if self._closed:
                raise RuntimeError('Robot runtime is already closed')
            if name in self._processes:
                raise RuntimeError('Process name already used in this session: ' + name)
            path = self.logs_dir / (name + '.log')
            with path.open('xb') as output:
                process = subprocess.Popen(list(argv), cwd=self.root, env=self.env,
                                           stdin=subprocess.DEVNULL, stdout=output,
                                           stderr=subprocess.STDOUT, start_new_session=True,
                                           close_fds=True)
            # A child may finish immediately; retain it and its exit code either way.
            row = dict(process=process, argv=list(argv), log=path,
                       identity=_identity(process.pid, self.marker), started_wall=time.time())
            self._processes[name] = row
            # /proc/environ may briefly lag exec immediately after Popen. Keep
            # the first verified identity so later cancellation stays pinned to
            # this child; do not turn a transient lookup miss into permanent
            # inability to send the client's graceful cancellation signal.
            deadline = time.monotonic() + .2
            while (row['identity'] is None and process.poll() is None
                   and time.monotonic() < deadline):
                time.sleep(.005)
                self._process_identity(row)
            try:
                self._write_processes()
            except BaseException:
                # The child may already have detached descendants or sent a
                # command. Close this owned session before reporting failure;
                # otherwise callers could retry alongside an untracked client.
                try:
                    self.close(graceful_seconds=1.0)
                except Exception:
                    pass  # Preserve the original registration error.
                raise
            return process

    def _write_processes(self):
        _save(self.session_dir / 'processes.json', {
            name: dict(pid=row['process'].pid, argv=row['argv'], log=str(row['log']),
                       started_wall=row['started_wall'], identity=row['identity'])
            for name, row in self._processes.items()})

    def snapshot(self):
        with self._mutex:
            values = {name: dict(state='disabled' if self.headless and name == 'rviz' else 'not_started',
                                 pid=None, returncode=None, log=None) for name in COMPONENTS}
            for name, row in self._processes.items():
                code = row['process'].poll()
                values[name] = dict(state='running' if code is None else 'exited',
                                    pid=row['process'].pid, returncode=code, log=str(row['log']))
            return values

    def log_tail(self, name, lines=80):
        with self._mutex:
            row = self._processes.get(name)
            if row is None:
                raise KeyError(name)
            path = row['log']
        lines = max(1, min(int(lines), 500))
        with path.open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - 128 * 1024))
            return '\n'.join(stream.read().decode('utf-8', errors='replace').splitlines()[-lines:])

    def signal_process(self, name, sig=signal.SIGINT):
        with self._mutex:
            row = self._processes.get(name)
            if row is None or row['process'].poll() is not None:
                return False
            identity = self._process_identity(row)
            if identity is None:
                return False
            return _signal(identity, self.marker, sig)

    def _process_identity(self, row):
        """Retry only missing identities of a still-live tracked child."""
        if row['identity'] is None and row['process'].poll() is None:
            row['identity'] = _identity(row['process'].pid, self.marker)
        return row['identity']

    def close(self, graceful_seconds=4.0):
        """Stop only this session, reaping tracked children and detached descendants."""
        if not math.isfinite(graceful_seconds) or graceful_seconds < 0:
            raise ValueError('graceful_seconds must be finite and nonnegative')
        with self._mutex:
            if self._closed:
                return self._receipt
            self._closed = True
            started = time.monotonic()
            signals = []
            try:
                # Registered wrappers get a chance to run traps. Reverse startup
                # order keeps the simulator last among the normal components.
                for name, row in reversed(tuple(self._processes.items())):
                    identity = self._process_identity(row)
                    if identity and row['process'].poll() is None:
                        if _signal(identity, self.marker, signal.SIGINT):
                            signals.append(dict(name=name, pid=identity['pid'], signal='SIGINT'))
                deadline = time.monotonic() + graceful_seconds
                while _owned(self.marker) and time.monotonic() < deadline:
                    for row in self._processes.values():
                        row['process'].poll()
                    time.sleep(.05)
                for sig, wait in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
                    remaining = _owned(self.marker)
                    if not remaining:
                        break
                    for identity in remaining:
                        if _signal(identity, self.marker, sig):
                            signals.append(dict(pid=identity['pid'], signal=sig.name))
                    deadline = time.monotonic() + wait
                    while _owned(self.marker) and time.monotonic() < deadline:
                        for row in self._processes.values():
                            row['process'].poll()
                        time.sleep(.05)
                for row in self._processes.values():
                    try:
                        row['process'].wait(timeout=.2)
                    except subprocess.TimeoutExpired:
                        pass
                remaining = _owned(self.marker)
                self._receipt = dict(closed_utc=datetime.now(timezone.utc).isoformat(),
                                     elapsed_wall_seconds=time.monotonic() - started,
                                     status='stopped' if not remaining else 'cleanup_incomplete',
                                     remaining_owned_processes=remaining, signals=signals,
                                     exit_codes={name: row['process'].poll()
                                                 for name, row in self._processes.items()},
                                     ownership='UID + unique inherited token + start_ticks + pidfd')
                if self.session_dir is not None:
                    _save(self.session_dir / 'shutdown.json', self._receipt)
                return self._receipt
            finally:
                self._release_locks()
