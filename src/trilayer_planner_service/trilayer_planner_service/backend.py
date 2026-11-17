# SPDX-License-Identifier: Apache-2.0
"""Snapshot-bound P6 Session adapter. No controller or host costmap mutation."""
import base64,hashlib,json,os
from pathlib import Path
from .protocol import ProtocolError
class Backend:
    def __init__(self,root,solver_root=None):
        self.root=Path(root);self.solver_root=solver_root;self.session=None
        self.context=None;self.generation=0;self.requests=0;self.cleanups=[]
    def close(self):
        if self.session is not None:
            result=self.session.close();self.cleanups.append(result);self.session=None
            if result.get('cleanup_errors') or result.get('worker_exit') not in (0,None):raise RuntimeError('SESSION_CLEANUP_FAILED')
        return {'closed':True,'cleanup':self.cleanups}
    def prepare(self,req):
        import numpy as np
        from PIL import Image
        c=req['context']
        try:raw=base64.b64decode(req['snapshot'],validate=True)
        except ValueError as exc:raise ProtocolError('INVALID_SNAPSHOT_ENCODING') from exc
        if len(raw)!=c['width']*c['height']:raise ProtocolError('SNAPSHOT_SIZE_MISMATCH')
        if hashlib.sha256(raw).hexdigest()!=c['costmap_sha256']:raise ProtocolError('COSTMAP_HASH_MISMATCH')
        costs=np.frombuffer(raw,dtype=np.uint8).reshape(c['height'],c['width']);hard=costs>=253
        if hashlib.sha256(hard.astype(np.uint8).tobytes()).hexdigest()!=c['hard_mask_sha256']:raise ProtocolError('HARD_MASK_HASH_MISMATCH')
        reused=self.session is not None and self.context==c
        if not reused:
            self.close();self.generation+=1
            generation=self.root/('generation-%04d'%self.generation);generation.mkdir()
            Image.fromarray(np.flipud(np.where(hard,0,254).astype(np.uint8))).save(generation/'map.pgm')
            mp=generation/'map.yaml'
            mp.write_text('image: map.pgm\nresolution: 0.05\norigin: '+json.dumps([*c['origin'],0.])+'\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            from trilayer_p6_runtime.api import MapOnlyRequest
            from .owner import RemoteSession
            self.session=RemoteSession(MapOnlyRequest(mp,tuple(req['start']),tuple(req['goal']),generation/'runtime',query_id=req['id']),solver_root=self.solver_root).open()
            actual=self.session.map_context()
            for key in ('frame_id','width','height','resolution','origin','hard_mask_sha256'):
                if actual[key]!=c[key]:
                    self.close();raise RuntimeError('PROJECTED_CONTEXT_MISMATCH:'+key)
            self.context=dict(c)
        return reused
    def dispatch(self,req):
        op=req['op'];self.requests+=1
        if op=='close':return {'ok':True,'path':[],**self.close()}
        if op=='status':return {'ok':True,'path':[],'session_generation':self.generation,'context':self.context,'active':self.session is not None}
        if op=='plan':
            reused=self.prepare(req)
            result=self.session.plan(start=req['start'],goal=req['goal'],query_id=req['id'])
        else:
            if self.session is None:raise ProtocolError('NO_ACTIVE_SESSION')
            if req['context']!=self.context:raise ProtocolError('CONTEXT_MISMATCH')
            result=self.session.update(**{k:req[k] for k in ('sequence','upsert','clear','observed_free','current_pose') if k in req})
            reused=True
        # Full native evidence stays private on disk, compact trusted path over IPC.
        file=self.root/('request-%06d.json'%self.requests)
        file.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        ok=bool(result.get('success'))
        path=result.get('path',[]) if ok else []
        audit_valid=result.get('result',{}).get('independent_audit',{}).get('valid') is True
        if op=='plan' and (not path or not audit_valid or type(result.get('dynamic_version')) is not int or result.get('trajectory_version')!=result.get('dynamic_version')):ok=False
        return {'ok':ok,'audit_valid':audit_valid,'path':path if ok else [],'frame_id':'map','context':self.context,
                'failure_code':result.get('result',{}).get('failure_code') or result.get('failure_code') or (None if ok else 'P6_REJECTED'),
                'session_generation':self.generation,'session_reused':reused,
                'native_pid':result.get('native_pid'),'dynamic_version':result.get('dynamic_version'),
                'trajectory_version':result.get('trajectory_version'),'sequence':result.get('sequence')}
