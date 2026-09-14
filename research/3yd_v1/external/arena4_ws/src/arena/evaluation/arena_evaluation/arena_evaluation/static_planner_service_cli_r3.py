"""Local Unix-socket JSON interface for the supervised static planner."""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import signal
import socket
import threading

from .static_planner_service_r3 import StaticPlannerSupervisor,MAX_RESULT_BYTES

MAX_REQUEST_BYTES=16*1024


class LocalServiceSocket:
    """At most eight accepted connections, one bounded JSON message per socket."""
    def __init__(self,supervisor,path,connections=8):
        if not 1<=connections<=8:raise ValueError('CONNECTION_LIMIT_INVALID')
        self.supervisor=supervisor;self.path=Path(path)
        self.slots=threading.BoundedSemaphore(connections)
        self.pool=ThreadPoolExecutor(max_workers=connections,thread_name_prefix='pln-local-api')
        self.socket=None;self.identity=None;self.stopping=threading.Event()

    def dispatch(self,request):
        if not isinstance(request,dict):raise ValueError('JSON_OBJECT_REQUIRED')
        command=request.get('command')
        if command=='status':return self.supervisor.status()
        if command=='cancel':
            rid=request.get('request_id')
            if not isinstance(rid,str) or not 1<=len(rid)<=128:raise ValueError('INVALID_REQUEST_ID')
            return {'cancel_requested':self.supervisor.cancel(rid),'request_id':rid}
        if command=='plan':
            # Received time is assigned by submit; its seven seconds includes
            # queue residence and all planning stages.
            future=self.supervisor.submit(request)
            return future.result(timeout=12.)
        raise ValueError('UNKNOWN_COMMAND')

    def _handle(self,connection):
        try:
            connection.settimeout(3.)
            buffer=bytearray()
            while b'\n' not in buffer:
                chunk=connection.recv(min(4096,MAX_REQUEST_BYTES+1-len(buffer)))
                if not chunk:raise ValueError('INCOMPLETE_REQUEST')
                buffer.extend(chunk)
                if len(buffer)>MAX_REQUEST_BYTES:raise ValueError('REQUEST_CAPACITY')
            line,extra=bytes(buffer).split(b'\n',1)
            if extra.strip():raise ValueError('ONE_REQUEST_PER_CONNECTION')
            try:
                request=json.loads(line,parse_constant=lambda value:(_ for _ in ()).throw(ValueError('NONFINITE_JSON')))
                result={'ok':True,'result':self.dispatch(request)}
            except (ValueError,RuntimeError,TimeoutError) as exc:
                result={'ok':False,'error':str(exc)}
            encoded=json.dumps(result,separators=(',',':'),allow_nan=False).encode()+b'\n'
            if len(encoded)>MAX_RESULT_BYTES:
                encoded=b'{"ok":false,"error":"RESULT_TRANSPORT_CAPACITY"}\n'
            connection.settimeout(3.);connection.sendall(encoded)
        except (OSError,ValueError):
            pass
        finally:
            connection.close();self.slots.release()

    def start(self):
        if self.socket is not None:raise RuntimeError('SOCKET_ALREADY_STARTED')
        if self.path.exists() or self.path.is_symlink():raise FileExistsError(self.path)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        # Protect the socket from its first instant of visibility.
        old=os.umask(0o177)
        try:self.socket.bind(str(self.path))
        finally:os.umask(old)
        stat=self.path.stat();self.identity=(stat.st_dev,stat.st_ino)
        self.socket.listen(8);self.socket.settimeout(.2)

    def serve(self):
        if self.socket is None:self.start()
        while not self.stopping.is_set():
            try:connection,_=self.socket.accept()
            except socket.timeout:continue
            except OSError:
                if self.stopping.is_set():break
                raise
            if not self.slots.acquire(blocking=False):
                connection.close();continue
            try:self.pool.submit(self._handle,connection)
            except BaseException:
                connection.close();self.slots.release();raise

    def close(self):
        self.stopping.set()
        if self.socket is not None:self.socket.close()
        self.pool.shutdown(wait=True,cancel_futures=False)
        try:
            stat=self.path.stat()
            if (stat.st_dev,stat.st_ino)==self.identity:self.path.unlink()
        except FileNotFoundError:pass


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True,help='EngineConfig JSON for one immutable static map')
    parser.add_argument('--output',type=Path,required=True,help='new service evidence directory')
    parser.add_argument('--socket',type=Path,required=True,help='new local Unix socket, mode 0600')
    args=parser.parse_args(argv)
    from .static_planner_engine_r3 import EngineConfig
    from dataclasses import asdict
    config=EngineConfig(**json.loads(args.config.read_text()))
    supervisor=StaticPlannerSupervisor(asdict(config),args.output)
    frontend=LocalServiceSocket(supervisor,args.socket)
    def stop(*_):frontend.stopping.set()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        frontend.start();supervisor.start();frontend.serve()
    finally:
        supervisor.close();frontend.close()
    return 0 if supervisor.state=='STOPPED' else 1


if __name__=='__main__':raise SystemExit(main())
