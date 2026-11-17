# SPDX-License-Identifier: Apache-2.0
import json,math,re
MAX_REQUEST=4*1024*1024
MAX_RESPONSE=8*1024*1024
class ProtocolError(ValueError):pass

def strict_json(data):
    def pairs(items):
        out={}
        for k,v in items:
            if k in out:raise ProtocolError('DUPLICATE_KEY')
            out[k]=v
        return out
    def bad(value):raise ProtocolError('NONFINITE_JSON')
    try:return json.loads(data,object_pairs_hook=pairs,parse_constant=bad)
    except (ValueError,UnicodeError) as exc:raise ProtocolError(str(exc)) from exc

def number(x):return type(x) in (int,float) and math.isfinite(x)
def safe_id(x):return isinstance(x,str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',x) is not None and '..' not in x
def pose(x):return isinstance(x,list) and len(x)==3 and all(number(v) for v in x)
def context(x):
    if not isinstance(x,dict) or set(x)!= {'frame_id','width','height','resolution','origin','hard_mask_sha256','costmap_sha256'}:raise ProtocolError('INVALID_CONTEXT_FIELDS')
    if x['frame_id']!='map' or any(type(x[k]) is not int or not 16<=x[k]<=1000000 for k in ('width','height')) or x['width']*x['height']>1000000:raise ProtocolError('INVALID_CONTEXT_MAP')
    if not number(x['resolution']) or x['resolution']!=.05 or not isinstance(x['origin'],list) or len(x['origin'])!=2 or not all(number(v) for v in x['origin']):raise ProtocolError('INVALID_CONTEXT_GEOMETRY')
    if not isinstance(x['hard_mask_sha256'],str) or not re.fullmatch('[a-f0-9]{64}',x['hard_mask_sha256']):raise ProtocolError('INVALID_CONTEXT_HASH')
    if not isinstance(x['costmap_sha256'],str) or not re.fullmatch('[a-f0-9]{64}',x['costmap_sha256']):raise ProtocolError('INVALID_COSTMAP_HASH')
    return x

def validate(req):
    if not isinstance(req,dict):raise ProtocolError('OBJECT_REQUIRED')
    if type(req.get('v')) is not int or req['v']!=1:raise ProtocolError('VERSION_REQUIRED')
    if not safe_id(req.get('id')):raise ProtocolError('INVALID_ID')
    op=req.get('op');base={'v','id','op'}
    fields={'plan':base|{'start','goal','context','snapshot'},'update':base|{'sequence','upsert','clear','observed_free','current_pose','context'},'status':base,'close':base}
    if not isinstance(op,str) or op not in fields:raise ProtocolError('UNSUPPORTED_OPERATION')
    if set(req)-fields[op]:raise ProtocolError('UNKNOWN_FIELD')
    if op=='plan':
        if not pose(req.get('start')) or not pose(req.get('goal')):raise ProtocolError('INVALID_POSE')
        context(req.get('context'))
        if not isinstance(req.get('snapshot'),str):raise ProtocolError('SNAPSHOT_REQUIRED')
    if op=='update':
        context(req.get('context'))
        if type(req.get('sequence')) is not int:raise ProtocolError('INVALID_SEQUENCE')
        if 'observed_free' in req and type(req['observed_free']) is not bool:raise ProtocolError('INVALID_OBSERVED_FREE')
        if 'current_pose' in req and req['current_pose'] is not None and not pose(req['current_pose']):raise ProtocolError('INVALID_CURRENT_POSE')
        clear=req.get('clear',[]);upsert=req.get('upsert',{})
        if not isinstance(clear,list) or len(clear)>64 or not all(safe_id(k) for k in clear) or len(set(clear))!=len(clear):raise ProtocolError('INVALID_CLEAR')
        if not isinstance(upsert,dict) or len(upsert)>64:raise ProtocolError('INVALID_UPSERT')
        for key,points in upsert.items():
            if not safe_id(key) or key in clear or not isinstance(points,list) or not 3<=len(points)<=128:raise ProtocolError('INVALID_POLYGON')
            if not all(isinstance(p,list) and len(p)==2 and all(number(v) for v in p) for p in points):raise ProtocolError('INVALID_POLYGON')
    return req

def read_line(sock,limit=MAX_REQUEST):
    data=bytearray()
    while True:
        chunk=sock.recv(min(65536,limit+1-len(data)))
        if not chunk:raise ProtocolError('UNTERMINATED_REQUEST')
        data.extend(chunk)
        if len(data)>limit:raise ProtocolError('MESSAGE_TOO_LARGE')
        if b'\n' in data:
            line,tail=bytes(data).split(b'\n',1)
            if tail:raise ProtocolError('ONE_REQUEST_PER_CONNECTION')
            return strict_json(line)

def encode(value):
    raw=(json.dumps(value,allow_nan=False,separators=(',',':'))+'\n').encode()
    if len(raw)>MAX_RESPONSE:raise ProtocolError('RESPONSE_TOO_LARGE')
    return raw
