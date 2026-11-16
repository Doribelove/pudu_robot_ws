"""Local continuous-curvature fitting of actual Smac geometry, with full checks.
No steering clipping, no change to minimum radius, no reference-only success.
"""
import time,math
import numpy as np
from arena_3d_v1.curvature_repair import violating_segments
from three_yd_v5.r10_connector import fit_connector
from arena_3d_v1.semantic_world import wrap,digest
from arena_evaluation.semantic_constraint_core import dubins_choices

def repair_path(world,channel,query,poses,mask,budget_s=5.):
    started=time.monotonic();deadline=started+budget_s;p=np.asarray(poses,float).copy();maximum=1/world.vehicle[2]
    log={'attempted':True,'success':False,'budget_s':budget_s,'original_hash':digest(p),'trials':[],'accepted_windows':[]}
    original=p.copy();attempts=0
    try:
        for window in range(16):
            bad,lengths=violating_segments(p,maximum)
            if not len(bad):
                log.update(success=True,repaired_hash=digest(p),wall_ms=(time.monotonic()-started)*1000)
                return p,log
            if time.monotonic()>deadline:raise TimeoutError('CHANNEL_CURVATURE_BUDGET')
            arc=np.r_[0,np.cumsum(lengths)];first=int(bad[0]);last=first
            # Work on one local cluster; distant turns do not create a huge fit.
            for k in bad[1:]:
                if arc[int(k)]-arc[last+1]>.35:break
                last=int(k)
            accepted=False
            for margin in [.4,.8,1.4]:
                i=max(0,int(np.searchsorted(arc,arc[first]-margin))-1)
                j=min(len(p)-1,int(np.searchsorted(arc,arc[last+1]+margin)))
                length=arc[j]-arc[i]
                if length>6:continue
                left=float(wrap(p[i,2]-p[i-1,2])/lengths[i-1]) if i else 0.
                right=float(wrap(p[j+1,2]-p[j,2])/lengths[j]) if j<len(p)-1 else 0.
                candidates=[]
                for radius in [.45,.6,.8]:
                    candidates.extend((sum(par)*radius,radius,word,par) for word,par in dubins_choices(p[i],p[j],radius))
                expanded=[]
                for clen,radius,word,par in sorted(candidates,key=lambda x:x[0])[:3]:
                    for slack in [.005,.02,.05,.1]:
                        if 'S' not in word:continue
                        adjusted=list(par);adjusted[word.index('S')]+=slack/radius
                        expanded.append((clen+slack,radius,word,adjusted))
                for clen,radius,word,par in expanded:
                    if time.monotonic()>deadline:raise TimeoutError('CHANNEL_CURVATURE_BUDGET')
                    if clen>min(6.,1.2*length+.2):continue
                    trial={'window':window,'anchors':[i,j],'radius':radius,'word':word,'original_length_m':float(length)};log['trials'].append(trial)
                    try:connector,fit=fit_connector(p[i],p[j],word,par,radius,left,right,deadline,maximum)
                    except ValueError as e:trial['rejection']=str(e);continue
                    full=np.r_[p[:i],connector,p[j+1:]]
                    ca=channel.audit(full)
                    if not ca['valid']:trial['rejection']=ca['failure'];continue
                    check=world.audit(query,full,mask)
                    if not check['valid']:trial['rejection']=check['failures'];continue
                    # Demand real progress at this splice; don't accept an
                    # endpoint change that merely relocates the same defect.
                    newbad,_=violating_segments(full,maximum)
                    if np.any(newbad< i+len(connector)-1):trial['rejection']='LOCAL_DISCONTINUITY_REMAINS';continue
                    p=full;accepted=True;trial['accepted']=True;trial['fit']=fit;log['accepted_windows'].append(trial);break
                if accepted:break
            if not accepted:log['failure']='NO_VALID_LOCAL_CONNECTOR';break
        else:log['failure']='LOCAL_WINDOW_COUNT_LIMIT'
    except TimeoutError as e:log['failure']=str(e)
    log['wall_ms']=(time.monotonic()-started)*1000
    return None,log
