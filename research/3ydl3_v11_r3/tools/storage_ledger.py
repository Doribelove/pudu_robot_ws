"""Read-only ndarray buffer inventory; never evaluates lazy properties.

This is diagnostic instrumentation, outside request timing. Array bytes are
not Python PSS or process-tree RSS. Aliased owners are counted once.
"""
from collections import deque
import types
import numpy as np


def inventory(roots, max_objects=100000, max_depth=12):
    queue=deque((name,obj,0) for name,obj in roots.items())
    seen=set();owners={};arrays=[];truncated=False
    while queue:
        path,obj,depth=queue.popleft()
        if isinstance(obj,np.ndarray):
            owner=obj
            while isinstance(getattr(owner,'base',None),np.ndarray):owner=owner.base
            if isinstance(getattr(owner,'base',None),(bytes,memoryview)):
                owner=owner.base
                while isinstance(owner,memoryview):owner=owner.obj
            key=id(owner)
            if key not in owners:
                size=int(owner.nbytes) if isinstance(owner,np.ndarray) else len(owner)
                owners[key]={'owner':len(owners)+1,'bytes':size,'aliases':[]}
            entry=owners[key]
            if len(entry['aliases'])<12:entry['aliases'].append(path)
            arrays.append({'path':path,'owner':entry['owner'],'logical_bytes':int(obj.nbytes),
                'shape':list(obj.shape),'dtype':str(obj.dtype),'writeable':bool(obj.flags.writeable),
                'owns_data':bool(obj.flags.owndata)})
            continue
        if obj is None or isinstance(obj,(str,bytes,int,float,bool,np.generic,types.ModuleType,types.FunctionType,type)):
            continue
        identity=id(obj)
        if identity in seen:continue
        seen.add(identity)
        if len(seen)>max_objects:truncated=True;break
        if depth>=max_depth:continue
        if isinstance(obj,dict):children=((str(k),v) for k,v in obj.items())
        elif isinstance(obj,(tuple,list,deque)):children=((str(i),v) for i,v in enumerate(obj))
        elif type(obj).__module__.startswith(('rclpy','logging','threading','concurrent')):continue
        elif hasattr(obj,'__dict__'):children=vars(obj).items()
        else:continue
        for name,value in children:queue.append((path+'.'+name,value,depth+1))
    buffers=sorted(owners.values(),key=lambda v:-v['bytes'])
    return {'schema':'ndarray-owner-ledger-v1','diagnostic_only':True,'properties_evaluated':False,
        'unique_array_buffer_bytes':sum(v['bytes'] for v in buffers),
        'logical_array_bytes_with_aliases':sum(v['logical_bytes'] for v in arrays),
        'objects_visited':len(seen),'truncated':truncated,'buffers':buffers,'arrays':arrays,
        'limits':'ndarray reachable buffer bytes only; excludes Python containers, ROS/C++ allocations and allocator retention'}
