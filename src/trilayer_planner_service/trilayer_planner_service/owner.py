# SPDX-License-Identifier: Apache-2.0
"""One interpreter owns exactly one native P6 Session."""
import json,os,socket,sys,subprocess
from pathlib import Path
from .protocol import read_line,encode,MAX_RESPONSE
class RemoteSession:
    def __init__(self,request,solver_root=None):
        self.request=request;self.solver_root=solver_root;self.process=None;self.sock=None
    def _call(self,op,**kwargs):
        self.sock.sendall(encode({'op':op,**kwargs}))
        data=read_line(self.sock,MAX_RESPONSE)
        if 'owner_error' in data:raise RuntimeError(data['owner_error'])
        return data['value']
    def open(self):
        from dataclasses import asdict
        root=Path(self.request.output_dir).parent
        config={'request':{k:str(v) if isinstance(v,Path) else v for k,v in asdict(self.request).items()},'solver_root':str(self.solver_root) if self.solver_root else None}
        cfg=root/'owner.json';cfg.write_text(json.dumps(config))
        self.sock,peer=socket.socketpair();self.sock.settimeout(60.)
        self.log=open(root/'owner.log','w')
        self.process=subprocess.Popen([sys.executable,'-m','trilayer_planner_service.owner',str(peer.fileno()),str(cfg)],pass_fds=(peer.fileno(),),stdin=subprocess.DEVNULL,stdout=self.log,stderr=subprocess.STDOUT,cwd=root,start_new_session=True)
        peer.close()
        try:self._call('open')
        except BaseException:self._abort();raise
        return self
    def _abort(self):
        import psutil
        if self.process is not None:
            try:children=psutil.Process(self.process.pid).children(recursive=True)
            except psutil.Error:children=[]
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
            for p in children:
                try:p.terminate()
                except psutil.Error:pass
            _,alive=psutil.wait_procs(children,timeout=2)
            for p in alive:
                try:p.kill()
                except psutil.Error:pass
            psutil.wait_procs(alive,timeout=2)
        if self.sock:self.sock.close()
        if hasattr(self,'log'):self.log.close()
    def plan(self,**kwargs):return self._call('plan',**kwargs)
    def update(self,**kwargs):return self._call('update',**kwargs)
    def map_context(self):return self._call('map_context')
    def close(self):
        try:
            value=self._call('close');self.process.wait(timeout=5)
            if self.process.returncode:raise RuntimeError('OWNER_EXIT_NONZERO')
            return value
        finally:self._abort()

def main():
    from trilayer_p6_runtime.api import MapOnlyRequest
    from trilayer_p6_runtime.session import P6Session
    sock=socket.socket(fileno=int(sys.argv[1]));cfg=json.loads(Path(sys.argv[2]).read_text())
    cfg['request']['map_yaml']=Path(cfg['request']['map_yaml']);cfg['request']['output_dir']=Path(cfg['request']['output_dir'])
    session=P6Session(MapOnlyRequest(**cfg['request']),solver_root=cfg['solver_root'])
    try:
        while True:
            req=read_line(sock,MAX_RESPONSE);op=req.pop('op')
            try:
                if op=='open':session.open();value={'opened':True}
                elif op in ('plan','update','map_context','close'):value=getattr(session,op)(**req)
                else:raise ValueError('INVALID_OWNER_OP')
                sock.sendall(encode({'value':value}))
            except BaseException as exc:
                sock.sendall(encode({'owner_error':type(exc).__name__+': '+str(exc)}))
                if not isinstance(exc,Exception):break
            if op=='close':break
    finally:session.close();sock.close()
if __name__=='__main__':main()
