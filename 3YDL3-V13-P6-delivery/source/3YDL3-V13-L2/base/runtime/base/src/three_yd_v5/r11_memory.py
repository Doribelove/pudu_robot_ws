"""Read-only ndarray ownership inventory, outside timed query execution."""
import sys
import numpy as np

def inventory(groups):
    roots={};seen=set();visits=0
    def walk(x,path,depth):
        nonlocal visits
        if x is None or depth>8:return
        visits+=1
        if isinstance(x,np.ndarray):
            owner=x
            while isinstance(owner.base,np.ndarray):owner=owner.base
            key=(int(owner.__array_interface__['data'][0]),int(owner.nbytes))
            item=roots.setdefault(key,{'bytes':int(owner.nbytes),'shape':list(owner.shape),'dtype':str(owner.dtype),'paths':[]})
            if len(item['paths'])<10:item['paths'].append(path)
            return
        if id(x) in seen:return
        seen.add(id(x))
        if isinstance(x,dict):
            for i,(k,v) in enumerate(x.items()):
                if i>=2000:break
                walk(v,path+'.'+str(k)[:80],depth+1)
        elif isinstance(x,(list,tuple)) and len(x)<=2000:
            for i,v in enumerate(x):walk(v,path+'.'+str(i),depth+1)
        elif hasattr(x,'__dict__') and (type(x).__module__.startswith(('arena_','three_yd')) or type(x).__name__=='SimpleNamespace'):
            for k,v in vars(x).items():
                if k in ('views','world','topology','controller','active_controller','client','stack','ctx','dynamic','fine'):continue
                walk(v,path+'.'+k,depth+1)
    for name,x in groups.items():walk(x,name,0)
    arrays=sorted(roots.values(),key=lambda x:x['bytes'],reverse=True)
    return {'unique_array_bytes':sum(x['bytes'] for x in arrays),'arrays':arrays,
            'object_visits':visits,'scope':'reachable numpy allocations; not total process memory or native planner allocations'}
