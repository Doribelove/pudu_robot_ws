"""Content-addressed preparation cache. No planned paths are stored or reused.

Map arrays are map-level; cover entries preserve exact query/ROI semantics.
Changing inputs, safe arrays or algorithm source produces a different key.
Files are verified before use. Corrupt entries fail closed; no pickle loading.
"""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import numpy as np
import yaml
from .output import sha256


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def algorithm_hash():
    return digest({p.name:sha256(p) for p in sorted(Path(__file__).parent.glob('*.py'))})


def atomic_bytes(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix='.prepare-',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as f:f.write(data)
        os.replace(tmp,path)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)


def cached_json(cache_dir,key,factory):
    started=time.monotonic();path=Path(cache_dir)/(key+'.json')
    if path.exists():
        envelope=json.loads(path.read_text())
        if envelope.get('key')!=key or envelope.get('sha256')!=digest(envelope.get('payload')):raise ValueError('CORRUPT_PREPARATION_CACHE')
        return envelope['payload'],{'cache_hit':True,'wall_s':time.monotonic()-started,'key':key}
    built=time.monotonic();value=factory();build_s=time.monotonic()-built
    # Normalize tuple/list representation so cold and cached results are identical.
    value=json.loads(json.dumps(value,ensure_ascii=False,allow_nan=False))
    atomic_bytes(path,json.dumps({'key':key,'sha256':digest(value),'payload':value},ensure_ascii=False,allow_nan=False).encode())
    return value,{'cache_hit':False,'wall_s':time.monotonic()-started,'build_s':build_s,'key':key}


def prepared_dataset(map_yaml,semantic_json,query_yaml,cache_dir):
    from .map_input import load_dataset
    started=time.monotonic();mp=Path(map_yaml).resolve();cfg=yaml.safe_load(mp.read_text());im=Path(cfg['image']);im=im if im.is_absolute() else mp.parent/im
    paths=[mp,im,Path(semantic_json).resolve(),Path(query_yaml).resolve()]
    key=digest({'kind':'map-arrays-v1','algorithm':algorithm_hash(),'files':{str(p):sha256(p) for p in paths}})
    directory=Path(cache_dir);directory.mkdir(parents=True,exist_ok=True);meta=directory/(key+'.json');array_path=directory/(key+'.npz')
    if meta.exists():
        record=json.loads(meta.read_text())
        if record.get('key')!=key or record.get('payload_hash')!=digest(record.get('payload')) or not array_path.exists() or sha256(array_path)!=record.get('array_hash'):
            raise ValueError('CORRUPT_MAP_CACHE')
        with np.load(array_path,allow_pickle=False) as z:arrays=[z[name] for name in ['free','safe','no_stopping']]
        for array in arrays:array.setflags(write=False)
        return (*arrays,*record['payload']),{'cache_hit':True,'wall_s':time.monotonic()-started,'key':key}
    value=load_dataset(map_yaml,semantic_json,query_yaml);fd,tmp=tempfile.mkstemp(prefix='.map-',dir=directory)
    try:
        with os.fdopen(fd,'wb') as f:np.savez_compressed(f,free=value[0],safe=value[1],no_stopping=value[2])
        os.replace(tmp,array_path)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)
    payload=list(value[3:]);record={'key':key,'array_hash':sha256(array_path),'payload_hash':digest(payload),'payload':payload}
    atomic_bytes(meta,json.dumps(record,ensure_ascii=False,allow_nan=False).encode())
    for array in value[:3]:array.setflags(write=False)
    return value,{'cache_hit':False,'wall_s':time.monotonic()-started,'key':key}


def prepared_scene(safe,no_stopping,config,query,evidence,cache_dir,**options):
    from .map_input import make_scene
    started=time.monotonic()
    key=digest({'kind':'query-roi-cover-v1','algorithm':algorithm_hash(),'config':config,'query':query,'evidence':evidence,'options':options,
                'safe_sha256':hashlib.sha256(safe.tobytes()).hexdigest(),'no_stopping_sha256':hashlib.sha256(no_stopping.tobytes()).hexdigest()})
    value,timing=cached_json(cache_dir,key,lambda:make_scene(safe,no_stopping,config,query,evidence,**options))
    timing['lookup_and_prepare_wall_s']=time.monotonic()-started
    return value,timing
