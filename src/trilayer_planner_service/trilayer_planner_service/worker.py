# SPDX-License-Identifier: Apache-2.0
import json,os,socket,sys,time,signal
from pathlib import Path
from .protocol import validate,read_line,encode
from .backend import Backend
from .resources import DiskSpaceLow,dispatch_checked

def main():
    config=json.loads(Path(sys.argv[1]).read_text());root=Path(config['output'])
    deadline=root/'deadline.json';backend=Backend(root,config.get('solver_root'))
    def timer(value):
        tmp=root/'deadline.tmp';tmp.write_text(json.dumps({'deadline_monotonic':value}));tmp.replace(deadline)
    def stop(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    listener.bind(str(root/'service.sock'));os.chmod(root/'service.sock',0o600);listener.listen(4);listener.settimeout(1)
    (root/'ready.json').write_text(json.dumps({'socket':str(root/'service.sock'),'pid':os.getpid()}))
    fatal=None;shutdown_reason=None
    try:
        while True:
            try:conn,_=listener.accept()
            except socket.timeout:continue
            close=False;req={}
            with conn:
                conn.settimeout(5)
                try:
                    req=validate(read_line(conn))
                    timer(time.monotonic()+config['request_timeout'])
                    value=dispatch_checked(backend,req,root,config.get('min_free_mib',512))
                    response={'v':1,'id':req['id'],**value}
                    close=req['op']=='close'
                except Exception as exc:
                    response={'v':1,'id':req.get('id') if isinstance(req,dict) else None,'ok':False,'path':[],'failure_code':'DISK_SPACE_LOW' if isinstance(exc,DiskSpaceLow) else type(exc).__name__+': '+str(exc)}
                    # Errors never return stale paths; unexpected runtime error closes the owner.
                    from .protocol import ProtocolError
                    if not isinstance(exc,(ProtocolError,socket.timeout)):
                        close=True;fatal=str(exc)
                    if isinstance(exc,DiskSpaceLow):
                        shutdown_reason='DISK_SPACE_LOW'
                        response.update(free_bytes=exc.free_bytes,required_bytes=exc.required_bytes)
                finally:
                    try:timer(None)
                    except OSError as exc:
                        close=True;fatal='DEADLINE_RECORD_WRITE_FAILED: '+str(exc)
                        response={'v':1,'id':req.get('id') if isinstance(req,dict) else None,'ok':False,'path':[],'failure_code':'AUDIT_STORAGE_WRITE_FAILED'}
                try:conn.sendall(encode(response))
                except (BrokenPipeError,ConnectionResetError,socket.timeout):pass
            if close:break
    except KeyboardInterrupt:shutdown_reason='SIGNAL'
    finally:
        try:timer(time.monotonic()+10)
        except OSError as exc:print('Cleanup deadline record failed: '+str(exc),file=sys.stderr,flush=True)
        try:cleanup=backend.close()
        except BaseException as exc:cleanup={'cleanup_errors':[str(exc)]}
        listener.close();(root/'service.sock').unlink(missing_ok=True)
        try:
            (root/'closed.json').write_text(json.dumps({'fatal':fatal,'shutdown_reason':shutdown_reason,'cleanup':cleanup},indent=2))
            timer(None)
        except OSError as exc:
            fatal='CLEANUP_RECORD_WRITE_FAILED: '+str(exc)
            print(fatal,file=sys.stderr,flush=True)
    return int(fatal is not None or bool(cleanup.get('cleanup_errors')))
if __name__=='__main__':raise SystemExit(main())
