#!/usr/bin/env python3
"""Local, single-session UI for the existing simulated search stack."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import signal
from subprocess import TimeoutExpired
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit
from uuid import uuid4
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
TERMINAL = {'completed', 'partial', 'failed', 'canceled'}


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


class Console:
    def __init__(self, runtime, monitor, startup_timeout=180, task_timeout=480):
        self.runtime, self.monitor = runtime, monitor
        self.startup_timeout, self.task_timeout = startup_timeout, task_timeout
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.phase, self.error = 'checking', None
        self.jobs, self.job, self.validation = {}, None, None
        self.artifacts, self.known_task_ids = {}, set()
        self.uncertain = False
        self.worker = None
        self.watchers = {}
        self.cancel_timeout = 8.0
        self.rooms_file = getattr(runtime, 'config', {}).get('rooms', 'src/semantic_search/config/rooms_aws_house_v4.json')
        self.rooms = json.loads((ROOT/self.rooms_file).read_text())
        self.started = time.monotonic()

    def healthy(self):
        return all(row['state'] == 'running' for name, row in self.runtime.snapshot().items()
                   if name in ('simulation', 'navigation', 'detector', 'search'))

    def readiness(self, telemetry):
        return (self.phase == 'ready' and not self.stopping.is_set() and not self.uncertain
                and self.healthy() and all(telemetry.get(key, False) for key in
                    ('simulation_ready', 'navigation_ready', 'perception_ready')))

    def snapshot(self):
        telemetry = self.monitor.snapshot()
        with self.lock:
            def public(row):
                if row is None:
                    return None
                result = {k: copy.deepcopy(v) for k, v in row.items() if not k.startswith('_')}
                result['elapsed_seconds'] = round(row.get('_ended', time.monotonic())-row['_started'], 2)
                return result
            busy = self.job is not None and self.job['state'] not in TERMINAL
            history = [public(job) for job in self.jobs.values() if job['state'] in TERMINAL][-30:][::-1]
            return dict(session_id=self.runtime.session_dir.name, phase=self.phase, error=self.error,
                        ready=self.readiness(telemetry) and not busy, busy=busy,
                        components=self.runtime.snapshot(), telemetry=telemetry,
                        task=public(self.job), history=history, validation=public(self.validation),
                        rooms=copy.deepcopy(self.rooms), rooms_frame_id="map", rooms_file=self.rooms_file,
                        output_directory=str(self.runtime.session_dir), control_uncertain=self.uncertain)

    def wait_ready(self, condition, phase):
        with self.lock:
            self.phase = phase
        deadline = time.monotonic()+self.startup_timeout
        while not self.stopping.wait(.2):
            states = self.runtime.snapshot()
            failed = [name for name, item in states.items()
                      if name in ('simulation','navigation','search','detector')
                      and item['state'] not in ('running','not_started')]
            if failed:
                raise RuntimeError('启动进程退出：'+', '.join(failed)+'；请查看对应日志。')
            if condition(self.monitor.snapshot()):
                return
            if time.monotonic() > deadline:
                raise TimeoutError('等待'+phase+'超时；请查看状态和组件日志。')
        raise InterruptedError('启动已停止')

    def start(self):
        self.monitor.start()
        self.worker = threading.Thread(target=self._startup, name='robot-startup', daemon=True)
        self.worker.start()

    def _startup(self):
        try:
            self.wait_ready(lambda s: s.get('discovery_ready', False), 'checking')
            # Give DDS a brief discovery window before starting any owned stack.
            if self.stopping.wait(2):
                return
            if self.monitor.foreign_stack_present():
                raise RuntimeError('当前ROS域已有机器人节点，请先关闭原运行栈，或通过 --domain 选择空闲域。')
            self.runtime.start_component('simulation')
            self.wait_ready(lambda s: s.get('simulation_ready', False), 'simulation')
            self.runtime.start_component('navigation')
            self.wait_ready(lambda s: s.get('navigation_ready', False), 'navigation')
            self.runtime.start_component('detector')
            self.runtime.start_component('search')
            if not self.runtime.headless:
                self.runtime.start_component('rviz')
            self.wait_ready(lambda s: all(s.get(key, False) for key in
                            ('simulation_ready','navigation_ready','perception_ready')), 'perception')
            with self.lock:
                self.phase = 'ready'
            print('机器人已就绪，可在网页发送任务。', flush=True)
            while not self.stopping.wait(.5):
                if not self.healthy():
                    raise RuntimeError('运行组件已退出；本次运行停止接收任务，请查看组件日志。')
        except InterruptedError:
            pass
        except Exception as error:
            with self.lock:
                self.phase, self.error = 'failed', str(error)
            print(str(error), file=sys.stderr, flush=True)
            try:
                self.cancel()
            finally:
                self.runtime.close()

    def normalize(self, payload):
        from semantic_search.semantic_map import parse_command
        if not isinstance(payload, dict):
            raise ApiError(400, '任务必须是JSON对象。')
        request_id = payload.get('request_id')
        if not isinstance(request_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}', request_id):
            raise ApiError(400, '请提供有效的request_id。')
        try:
            if 'command' in payload:
                if set(payload)-{'command','request_id'} or not isinstance(payload['command'], str):
                    raise ValueError('文字任务字段无效')
                text = payload['command'].strip()
                if not text or len(text) > 500:
                    raise ValueError('任务文字长度应为1–500字符')
            else:
                if set(payload) != {'mode','room','category','request_id'}:
                    raise ValueError('需要mode、room和category')
                text = json.dumps({k:payload[k] for k in ('mode','room','category')}, ensure_ascii=False)
            mode, room, category = parse_command(text, self.rooms)
            if mode not in ('find','count'):
                raise ValueError('停止任务请使用停止按钮')
            canonical = json.dumps(dict(mode=mode,room=room,category=category),ensure_ascii=False,sort_keys=True)
            return request_id, text, canonical
        except (ValueError, KeyError, TypeError) as error:
            raise ApiError(400, str(error)) from error

    def submit(self, payload):
        request_id, text, command = self.normalize(payload)
        with self.lock:
            if request_id in self.jobs:
                if self.jobs[request_id]['_signature'] != command:
                    raise ApiError(409, '同一request_id不能用于不同任务。')
                return self.snapshot()
            return self._start_job(request_id, 'search', text, command)

    def validate(self):
        with self.lock:
            return self._start_job(uuid4().hex, 'validation', '传感器与TF检查（10秒采样）', 'validate')

    def _start_job(self, request_id, kind, text, command):
        if self.job and self.job['state'] not in TERMINAL:
            raise ApiError(409, '已有任务执行中，请等待结束或停止。')
        if not self.readiness(self.monitor.snapshot()):
            raise ApiError(409, self.error or '机器人尚未就绪，请等待或检查状态。')
        folder = self.runtime.session_dir/'requests'/request_id
        row = dict(id=request_id,kind=kind,command=text,state='starting',result=None,error=None,
                   artifacts=[],save_status='pending',_started=time.monotonic(),_signature=command,
                   _folder=folder,_process_name='request_'+hashlib.sha256(request_id.encode()).hexdigest()[:32])
        self.jobs[request_id] = self.job = row
        if kind == 'validation':
            self.validation = row
        launch_attempted = False
        try:
            folder.mkdir(parents=True, exist_ok=False)
            write_json(folder/'request.json', dict(request_id=request_id,kind=kind,input=text,
                       executed_command=command,created_utc=datetime.now(timezone.utc).isoformat()))
            if kind == 'search':
                argv = [sys.executable,str(ROOT/'scripts/run_search.py'),command,'--timeout',str(self.task_timeout),
                        '--output',str(folder/'client_result.json'),'--diagnostics-output',str(folder/'diagnostics.json'),
                        '--initial-state-output',str(folder/'initial_state.json')]
                for old_id in sorted(self.known_task_ids):
                    argv += ['--exclude-task-id',old_id]
            else:
                argv = [sys.executable,str(ROOT/'scripts/validate_sensors.py'),'--seconds','10',
                        '--sensor-reliability','reliable','--output',str(folder/'sensors')]
            launch_attempted = True
            process = self.runtime.start_process(row['_process_name'], argv)
            row['_process'] = process
            row['state'] = 'running'
            watcher = threading.Thread(target=self._watch_job,args=(row,),daemon=True,name='job-'+request_id)
            watcher.start()
            self.watchers[request_id] = watcher
        except Exception as error:
            # A start failure may happen after Popen succeeds or a command is
            # already published. Never accept another search in that ambiguity.
            if launch_attempted:
                self.uncertain = kind == 'search' or self.uncertain
                try:
                    self.runtime.close()
                except Exception as cleanup_error:
                    error = RuntimeError(str(error)+'；清理失败：'+str(cleanup_error))
            row.update(state='failed',error=str(error),_ended=time.monotonic(),save_status='missing')
            self._save_job(row)
            raise ApiError(500, '无法启动任务：'+str(error)) from error
        return self.snapshot()

    def _register(self, path, name):
        path = Path(path).resolve()
        if not path.is_relative_to(self.runtime.session_dir.resolve()) or not path.is_file():
            return None
        key = hashlib.sha256(str(path).encode()).hexdigest()[:32]
        with self.lock:
            self.artifacts[key] = path
        return dict(name=name,url='/api/artifact?id='+key,kind=path.suffix.lstrip('.'))

    def _finish_artifacts(self, row, result):
        folder = row['_folder']
        paths = [(folder/'request.json','任务请求')]
        if row['kind']=='validation':
            paths += [(folder/'sensors/sensors.json','传感器报告'),(folder/'sensors/rgb.png','采样图像')]
        else:
            paths += [(folder/'client_result.json','结果JSON'),(folder/'diagnostics.json','客户端诊断'),
                      (folder/'initial_state.json','初始位姿')]
            if result and result.get('task_id'):
                deadline = time.monotonic()+5
                persisted = None
                while time.monotonic()<deadline:
                    for p in (self.runtime.session_dir/'tasks').glob('*/result.json'):
                        try:
                            if json.loads(p.read_text()).get('task_id')==result['task_id']:
                                persisted=p.parent
                                break
                        except (OSError, ValueError):
                            continue
                    if persisted and all((persisted/name).is_file() for name in ('observations.json','semantic_map.svg')):
                        break
                    time.sleep(.1)
                if persisted:
                    paths += [(persisted/'result.json','持久化结果'),(persisted/'observations.json','逐帧观测'),
                              (persisted/'semantic_map.svg','编号地图SVG')]
                else:
                    paths += [(folder/'missing_result.json','持久化结果（缺失）')]
        artifacts=[self._register(p,label) for p,label in paths]
        return [a for a in artifacts if a], 'saved' if all(artifacts) else 'partial'

    def _watch_job(self, row):
        process=row['_process']
        limit=self.task_timeout+50 if row['kind']=='search' else 50
        folder=row['_folder'];result=None;error=None
        try:
            while process.poll() is None:
                now=time.monotonic()
                with self.lock:cancel_started=row.get('_cancel_started')
                if (now-row['_started']>=limit or
                        cancel_started is not None and now-cancel_started>=self.cancel_timeout):
                    break
                time.sleep(.1)
            if process.poll() is None:
                # User cancellation has its own short deadline, independent of
                # the normal task's potentially several-minute timeout.
                with self.lock:already_canceling=row.get('_cancel_started') is not None
                steps = [(signal.SIGTERM,3), (signal.SIGKILL,3)] if already_canceling else [
                    (signal.SIGINT,self.cancel_timeout), (signal.SIGTERM,3), (signal.SIGKILL,3)]
                for sig, timeout in steps:
                    self._signal_job(row,sig)
                    try:
                        process.wait(timeout=timeout)
                        break
                    except TimeoutExpired:
                        continue
                if process.poll() is None:
                    raise RuntimeError('任务客户端在停止期限内未退出；已禁用新任务。')
            result_path=folder/('client_result.json' if row['kind']=='search' else 'sensors/sensors.json')
            if result_path.exists():
                result=json.loads(result_path.read_text())
            if row['kind']=='search':
                diagnostics=json.loads((folder/'diagnostics.json').read_text()) if (folder/'diagnostics.json').exists() else {}
                status=(result or {}).get('status')
                if status in ('found','completed','not_found') and process.returncode in (0,1):state='completed'
                elif status=='partial':state='partial'
                elif status=='canceled' or (not diagnostics.get('command_sent',True) and row['state']=='canceling'):state='canceled'
                else:state='failed'
                if result and result.get('task_id'):
                    with self.lock:self.known_task_ids.add(result['task_id'])
                if not result and diagnostics.get('command_sent',True):
                    with self.lock:self.uncertain=True
                    error='任务结束状态尚未确认，已阻止新任务；请停止本次运行并检查日志。'
                elif state=='failed':
                    error='；'.join(diagnostics.get('errors',[])) or ('任务结果：'+str(status or diagnostics.get('outcome','client_error')))
            else:
                state='completed' if result and result.get('passed') and process.returncode==0 else ('canceled' if row['state']=='canceling' else 'failed')
                if state=='failed':error='传感器检查未通过，请查看报告和任务日志。'
            artifacts,save_status=self._finish_artifacts(row,result)
            if save_status!='saved':error=(error+'；' if error else '')+'部分结果文件尚未保存，请查看搜索日志。'
        except Exception as caught:
            state,error,artifacts,save_status='failed',str(caught),[],'missing'
            if row['kind']=='search':
                with self.lock:self.uncertain=True
        with self.lock:
            if row.get('_finalized'):
                return
            row.update(state=state,result=result,error=error,artifacts=artifacts,save_status=save_status,
                       returncode=process.returncode,_ended=time.monotonic())
            self._save_job(row)

    def _save_job(self,row):
        """Best-effort receipt; a write failure must remain visible in the UI."""
        try:
            write_json(row['_folder']/'console_result.json',
                       {k:v for k,v in row.items() if not k.startswith('_')})
        except Exception as error:
            row['save_status']='missing'
            row['error']=(row.get('error')+'；' if row.get('error') else '')+'保存任务回执失败：'+str(error)

    def _signal_job(self,row,sig):
        try:
            delivered=self.runtime.signal_process(row['_process_name'],sig)
            detail=None
        except Exception as error:
            delivered,detail=False,str(error)
        if not delivered:
            process=row.get('_process')
            if process is not None and process.poll() is None:
                with self.lock:
                    if row['kind']=='search':self.uncertain=True
                    row['error']='未能确认发送'+sig.name+'；将在停止期限内继续清理。'+(detail or '')
        return delivered

    def cancel(self):
        with self.lock:
            if self.job and self.job['state'] not in TERMINAL:
                if self.job['state']!='canceling':
                    self.job['state']='canceling'
                    self.job['_cancel_started']=time.monotonic()
                    self.job['cancel_signal_delivered']=self._signal_job(self.job,signal.SIGINT)
            return self.snapshot()

    def log(self, component):
        if component=='task':
            with self.lock:component=self.job['_process_name'] if self.job else ''
        if not component:return ''
        try:
            return self.runtime.log_tail(component)
        except KeyError:
            return '组件尚未启动。'

    def close(self):
        self.stopping.set()
        with self.lock:self.phase='stopping'
        self.cancel()
        # First let the client collect cancellation evidence while ROS is live.
        deadline=time.monotonic()+self.cancel_timeout
        for watcher in list(self.watchers.values()):
            watcher.join(timeout=max(0,deadline-time.monotonic()))
        if self.worker:self.worker.join(timeout=2)
        errors=[]
        try:
            receipt=self.runtime.close()
            if receipt and receipt.get('status')=='cleanup_incomplete':
                errors.append('部分本会话进程未退出，请查看shutdown.json。')
        except Exception as error:
            errors.append('关闭运行栈失败：'+str(error))
        finally:
            try:self.monitor.close()
            except Exception as error:errors.append('关闭监测失败：'+str(error))
        # Runtime cleanup stops the remaining clients. Await result collection
        # before writing the final snapshot or letting daemon workers disappear.
        deadline=time.monotonic()+8
        for watcher in list(self.watchers.values()):
            watcher.join(timeout=max(0,deadline-time.monotonic()))
        with self.lock:
            for row in self.jobs.values():
                if row['state'] not in TERMINAL:
                    row.update(state='failed',error='关闭期间未收到任务结束回执，执行状态未确认。',
                               save_status='partial',_ended=time.monotonic(),_finalized=True)
                    if row['kind']=='search':self.uncertain=True
                    self._save_job(row)
            self.phase='failed' if errors else 'stopped'
            if errors:self.error='；'.join(errors)
        try:write_json(self.runtime.session_dir/'console_final.json',self.snapshot())
        except Exception as error:print('保存最终状态失败：'+str(error),file=sys.stderr,flush=True)


def handler_for(console):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def reply(self,status,payload):
            body=json.dumps(payload,ensure_ascii=False,allow_nan=False).encode()
            self.send_bytes(status,body,'application/json; charset=utf-8')

        def send_bytes(self,status,body,mime):
            self.send_response(status)
            self.send_header('Content-Type',mime)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass

        def same_origin(self,mutation=False):
            hosts={'127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)}
            if self.headers.get('Host') not in hosts:
                raise ApiError(403,'仅允许本机控制台访问。')
            origin=self.headers.get('Origin')
            if origin and origin not in {'http://'+host for host in hosts}:
                raise ApiError(403,'请求来源不匹配。')
            if mutation and self.headers.get_content_type()!='application/json':
                raise ApiError(415,'请使用application/json。')

        def do_GET(self):
            try:
                self.same_origin()
                url=urlsplit(self.path)
                if url.path=='/api/state':return self.reply(200,console.snapshot())
                if url.path=='/api/map':return self.reply(200,console.monitor.map_snapshot())
                if url.path=='/api/log':
                    name=parse_qs(url.query).get('component',['search'])[0]
                    if name not in ('simulation','navigation','detector','search','rviz','task'):
                        raise ApiError(400,'未知日志组件。')
                    return self.reply(200,{'text':console.log(name)})
                if url.path=='/api/artifact':
                    key=parse_qs(url.query).get('id',[''])[0]
                    with console.lock:path=console.artifacts.get(key)
                    if path is None or not path.resolve().is_relative_to(console.runtime.session_dir.resolve()):
                        raise ApiError(404,'结果文件不存在。')
                    return self.send_bytes(200,path.read_bytes(),mimetypes.guess_type(path.name)[0] or 'application/octet-stream')
                static={'/':'index.html','/index.html':'index.html','/app.js':'app.js','/style.css':'style.css'}
                if url.path not in static:raise ApiError(404,'路径不存在。')
                path=ROOT/'ui'/static[url.path]
                return self.send_bytes(200,path.read_bytes(),mimetypes.guess_type(path.name)[0] or 'text/plain')
            except ApiError as error:self.reply(error.status,{'error':str(error)})
            except Exception as error:self.reply(500,{'error':str(error)})

        def do_POST(self):
            try:
                self.same_origin(mutation=True)
                try:length=int(self.headers.get('Content-Length','0'))
                except ValueError:raise ApiError(400,'请求长度无效。')
                if not 0<length<=4096:raise ApiError(413,'请求长度超限。')
                try:data=json.loads(self.rfile.read(length))
                except (ValueError,UnicodeError):raise ApiError(400,'无效JSON。')
                if not isinstance(data,dict):raise ApiError(400,'请求必须为JSON对象。')
                if self.path=='/api/task':result=console.submit(data)
                elif self.path=='/api/cancel':result=console.cancel()
                elif self.path=='/api/validate':result=console.validate()
                else:raise ApiError(404,'路径不存在。')
                self.reply(200,result)
            except ApiError as error:self.reply(error.status,{'error':str(error)})
            except Exception as error:self.reply(500,{'error':str(error)})
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,default=66)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--headless',action='store_true',help='不启动Gazebo GUI和RViz，保留网页与真实仿真')
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--startup-timeout',type=float,default=180)
    parser.add_argument('--task-timeout',type=float,default=480)
    args=parser.parse_args()
    if not 1024<=args.port<=65535 or not 0<=args.domain<=232:
        parser.error('port应为1024–65535，domain应为0–232')
    if not all(math.isfinite(x) and x>0 for x in (args.startup_timeout,args.task_timeout)):
        parser.error('超时必须是有限正数')
    from robot_runtime import Runtime
    from robot_monitor import Monitor
    runtime=console=server=None
    shutdown=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *unused:shutdown.set())
    try:
        runtime=Runtime(ROOT,domain=args.domain,headless=args.headless)
        # Only ROS selectors belong to the parent. Browser processes must not
        # inherit the ownership token used to clean up this robot's children.
        for key in ('ROS_DOMAIN_ID','SEMANTIC_ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION'):
            os.environ[key]=runtime.env[key]
        console=Console(runtime,Monitor(),args.startup_timeout,args.task_timeout)
        server=ThreadingHTTPServer(('127.0.0.1',args.port),handler_for(console))
        server.daemon_threads=True
        server.timeout=.3
        url='http://127.0.0.1:'+str(args.port)
        print('控制台：'+url+'\n本次记录：'+str(runtime.session_dir)+'\n按 Ctrl+C 停止整套系统。',flush=True)
        console.start()
        if not args.no_browser:
            threading.Thread(target=lambda:webbrowser.open(url),daemon=True).start()
        while not shutdown.is_set():server.handle_request()
    except Exception as error:
        print('启动失败：'+str(error),file=sys.stderr,flush=True)
        return 1
    finally:
        if console:console.close()
        elif runtime:runtime.close()
        if server:server.server_close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
