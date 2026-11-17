"""Strict static semantic input subset, validated before P6 activation/ROS launch.

Only axis-aligned rectangular lane/junction/parking regions and explicitly
supported traffic rules are accepted. Unsupported research schema is rejected.
"""
from pathlib import Path
import hashlib,json,math,io

def _fail(message):raise ValueError('Semantic input: '+message)
def _numeric(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
def _keys(v,allowed,required,label):
    if not isinstance(v,dict) or set(v)-set(allowed) or set(required)-set(v):_fail(label+' has missing or unsupported fields')
def _text(v):return isinstance(v,str) and bool(v.strip())
def _load(path):
    p=Path(path)
    if not p.is_absolute() or not p.is_file():_fail('JSON inputs must be existing absolute file paths')
    raw=p.read_bytes()
    if len(raw)>8*1024*1024:_fail('JSON input exceeds 8 MiB')
    def pairs(items):
        result={}
        for key,value in items:
            if key in result:_fail('duplicate JSON key '+key)
            result[key]=value
        return result
    try:doc=json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda x:_fail('nonfinite JSON number '+x))
    except (UnicodeError,json.JSONDecodeError) as exc:_fail('invalid JSON: '+str(exc))
    return raw,doc

def read_inputs(request):
    """Validate and retain exact input bytes; no file creation or runtime import."""
    import yaml
    from PIL import Image
    mp=Path(request.map_yaml);map_raw=mp.read_bytes();cfg=yaml.safe_load(map_raw)
    image=Path(cfg['image']);image=image if image.is_absolute() else mp.parent/image
    image_raw=image.read_bytes()
    if len(image_raw)>32*1024*1024:_fail('image input exceeds 32 MiB')
    with Image.open(io.BytesIO(image_raw)) as im:width,height=im.size;im.load()
    sem_raw,sem=_load(request.semantic_json);top_raw,top=_load(request.topology_json);parking_raw,parking=_load(request.parking_access_json)
    _keys(sem,{'schema_version','frame_id','resolution','origin','width','height','source_pdmap_hash','features','traffic_rules','diagnostics','unrecognized_fields','semantic_map_hash'}, {'schema_version','frame_id','resolution','origin','width','height','source_pdmap_hash','features','traffic_rules'},'semantic document')
    if sem['schema_version']!='SemanticMapV1' or sem['frame_id']!='map':_fail('unsupported semantic schema/frame')
    if not _numeric(sem['resolution']) or sem['resolution']!=cfg['resolution']:_fail('semantic resolution mismatch')
    if not isinstance(sem['origin'],list) or len(sem['origin'])!=3 or not all(_numeric(x) for x in sem['origin']) or sem['origin']!=cfg['origin']:_fail('semantic origin mismatch')
    if type(sem['width']) is not int or type(sem['height']) is not int or (sem['width'],sem['height'])!=(width,height):_fail('semantic dimensions mismatch')
    image_hash=hashlib.sha256(image_raw).hexdigest()
    if sem['source_pdmap_hash']!=image_hash:_fail('semantic source image hash mismatch')
    if sem.get('diagnostics',[]) or sem.get('unrecognized_fields',[]):_fail('unresolved diagnostics/fields unsupported')
    features=sem['features']
    if not isinstance(features,list) or not 1<=len(features)<=128:_fail('require 1..128 explicit features')
    by_name={};by_id={};boxes={}
    for f in features:
        _keys(f,{'semantic_id','semantic_class','geometry_type','coordinates','hard','soft','non_stopping','direction_rule','priority','source_field','properties'}, {'semantic_id','semantic_class','geometry_type','coordinates','properties'},'feature')
        sid=f['semantic_id'];kind=f['semantic_class']
        if not _text(sid) or sid in by_id:_fail('duplicate/invalid semantic ID')
        if kind not in ('lane','junction_area','parking_area') or f['geometry_type']!='polygon':_fail('unsupported feature type')
        for key in ('hard','soft','non_stopping'):
            if key in f and f[key] is not False:_fail('unsupported feature '+key)
        if f.get('direction_rule','none')!='none' or type(f.get('priority',0)) is not int or f.get('priority',0)!=0 or f.get('source_field','')!='':_fail('unsupported feature policy')
        _keys(f['properties'],{'source_name'},{'source_name'},'feature properties');name=f['properties']['source_name']
        if not _text(name) or name in by_name:_fail('duplicate/invalid source_name')
        pts=f['coordinates']
        if not isinstance(pts,list) or len(pts)!=4 or any(not isinstance(p,list) or len(p)!=2 or not all(_numeric(x) for x in p) for p in pts):_fail('require four numeric rectangle vertices')
        xs=sorted({p[0] for p in pts});ys=sorted({p[1] for p in pts})
        if len(xs)!=2 or len(ys)!=2 or len({tuple(p) for p in pts})!=4:_fail('only nondegenerate axis-aligned rectangles supported')
        if any((a[0]==b[0])==(a[1]==b[1]) for a,b in zip(pts,pts[1:]+pts[:1])):_fail('rectangle polygon ordering is crossed or degenerate')
        x0,y0,x1,y1=xs[0],ys[0],xs[1],ys[1]
        ox,oy,_=cfg['origin']
        if not(ox<=x0<x1<=ox+width*.05 and oy<=y0<y1<=oy+height*.05):_fail('feature extends outside map')
        if kind=='lane' and x1-x0==y1-y0:_fail('square lane has ambiguous travel axis')
        boxes[name]=(x0,y0,x1,y1);by_name[name]=f;by_id[sid]=f
    names=list(boxes)
    for i,a in enumerate(names):
        ax,ay,bx,by=boxes[a]
        for b in names[i+1:]:
            cx,cy,dx,dy=boxes[b]
            if min(bx,dx)>max(ax,cx) and min(by,dy)>max(ay,cy):_fail('overlapping semantic region interiors unsupported')
    _keys(top,{'regions','edges'},{'regions','edges'},'topology')
    if not isinstance(top['regions'],list) or len(top['regions'])!=len(features):_fail('topology must cover each feature exactly once')
    regions={}
    for r in top['regions']:
        _keys(r,{'id','name','kind'},{'id','name','kind'},'region')
        if type(r['id']) is not int or not 1<=r['id']<=32767 or r['id'] in regions:_fail('invalid/duplicate region ID')
        if not isinstance(r['name'],str) or r['name'] not in by_name or r['kind']!=by_name[r['name']]['semantic_class']:_fail('region/feature name or class mismatch')
        regions[r['id']]=r
    if len({r['name'] for r in regions.values()})!=len(features):_fail('duplicate region names')
    if not isinstance(top['edges'],list) or len(top['edges'])>512:_fail('invalid edges')
    edges=set()
    for e in top['edges']:
        _keys(e,{'kind','regions','geometry'},{'kind','regions','geometry'},'edge')
        ids=e['regions'];points=e['geometry']
        if e['kind']!='shared_outline' or not isinstance(ids,list) or len(ids)!=2 or any(type(x) is not int or x not in regions for x in ids) or ids[0]==ids[1]:_fail('invalid shared edge reference')
        key=tuple(sorted(ids))
        if key in edges:_fail('duplicate edge')
        edges.add(key)
        if not isinstance(points,list) or len(points)!=2 or any(not isinstance(p,list) or len(p)!=2 or not all(_numeric(x) for x in p) for p in points) or points[0]==points[1]:_fail('edge must be two distinct numeric points')
        a=boxes[regions[ids[0]]['name']];b=boxes[regions[ids[1]]['name']]
        def on_border(p,box):
            x,y=p;x0,y0,x1,y1=box
            return x0<=x<=x1 and y0<=y<=y1 and (x in (x0,x1) or y in (y0,y1))
        if not all(on_border(pt,a) and on_border(pt,b) for pt in points):_fail('edge geometry is not on both region boundaries')
    rules=sem['traffic_rules'];_keys(rules,{'one_way','forbidden_transitions'},set(),'traffic_rules')
    if not isinstance(rules.get('one_way',[]),list):_fail('one_way must be a list')
    for rule in rules.get('one_way',[]):
        _keys(rule,{'semantic_id','yaw'},{'semantic_id','yaw'},'one_way')
        if not isinstance(rule['semantic_id'],str):_fail('one_way ID must be text')
        f=by_id.get(rule['semantic_id'])
        if f is None or f['semantic_class']!='lane' or not _numeric(rule['yaw']):_fail('invalid one_way lane/yaw')
        x0,y0,x1,y1=boxes[f['properties']['source_name']]
        if abs(math.sin(rule['yaw']) if x1-x0>y1-y0 else math.cos(rule['yaw']))>1e-9:_fail('one_way must align with lane axis')
    if not isinstance(rules.get('one_way',[]),list) or len({r['semantic_id'] for r in rules.get('one_way',[])})!=len(rules.get('one_way',[])):_fail('invalid/duplicate one_way list')
    transitions=rules.get('forbidden_transitions',[])
    if not isinstance(transitions,list):_fail('forbidden_transitions must be a list')
    transition_seen=set()
    sid_to_region={by_name[r['name']]['semantic_id']:rid for rid,r in regions.items()}
    for t in transitions:
        if not isinstance(t,list) or len(t)!=3 or any(not isinstance(x,str) or x not in by_id for x in t) or len(set(t))!=3:_fail('invalid forbidden transition IDs')
        if [by_id[x]['semantic_class'] for x in t]!=['lane','junction_area','lane']:_fail('forbidden transition must be lane/junction/lane')
        if tuple(t) in transition_seen:_fail('duplicate forbidden transition')
        transition_seen.add(tuple(t))
        if any(tuple(sorted((sid_to_region[a],sid_to_region[b]))) not in edges for a,b in zip(t,t[1:])):_fail('forbidden transition does not follow topology edges')
    _keys(parking,{'bindings'},{'bindings'},'parking access')
    if not isinstance(parking['bindings'],list):_fail('bindings must be a list')
    seen=set()
    for b in parking['bindings']:
        _keys(b,{'query_id','endpoint','parking_region_id','road_id','anchor_xy','anchor_tolerance_m','travel_sides','allow_local_cross_road'}, {'query_id','endpoint','parking_region_id','road_id','anchor_xy','anchor_tolerance_m','travel_sides','allow_local_cross_road'},'parking binding')
        if b['query_id']!=request.query_id or b['endpoint'] not in ('start','goal'):_fail('parking binding must identify this query start/goal')
        if b['endpoint'] in seen:_fail('duplicate endpoint ownership');
        seen.add(b['endpoint'])
        for key,kind in [('parking_region_id','parking_area'),('road_id','lane')]:
            if type(b[key]) is not int or b[key] not in regions or regions[b[key]]['kind']!=kind:_fail('invalid parking/road binding ID')
        if not isinstance(b['anchor_xy'],list) or len(b['anchor_xy'])!=2 or not all(_numeric(x) for x in b['anchor_xy']):_fail('invalid parking anchor')
        box=boxes[regions[b['parking_region_id']]['name']];x,y=b['anchor_xy']
        if not(box[0]<=x<=box[2] and box[1]<=y<=box[3]):_fail('anchor outside parking region')
        if not _numeric(b['anchor_tolerance_m']) or not 0<b['anchor_tolerance_m']<=.25:_fail('parking anchor tolerance must be in (0,.25]')
        sides=b['travel_sides']
        if not isinstance(sides,list) or not sides or any(x not in ('top','bottom','left','right') for x in sides) or len(set(sides))!=len(sides):_fail('invalid travel_sides')
        if type(b['allow_local_cross_road']) is not bool:_fail('allow_local_cross_road must be boolean')
    if 'semantic_map_hash' in sem:
        normalized=dict(sem);expected=normalized.pop('semantic_map_hash')
        normalized['origin']=[float(x) for x in sem['origin']];normalized['resolution']=float(sem['resolution'])
        normalized.setdefault('diagnostics',[]);normalized.setdefault('unrecognized_fields',[])
        defaults={'hard':False,'soft':False,'non_stopping':False,'direction_rule':'none','priority':0,'source_field':''}
        normalized['features']=[{**defaults,**f,'coordinates':[[float(x) for x in pt] for pt in f['coordinates']]} for f in features]
        actual=hashlib.sha256(json.dumps(normalized,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
        if not isinstance(expected,str) or expected!=actual:_fail('semantic content hash mismatch')
    return {'map_yaml':map_raw,'map_image':image_raw,'semantic.json':sem_raw,'topology.json':top_raw,'parking_access.json':parking_raw},cfg

def snapshot_inputs(out,blobs,cfg):
    """Write a private snapshot from retained bytes, never reopen original inputs."""
    import yaml
    folder=Path(out)/'inputs';folder.mkdir()
    saved=dict(blobs);saved['map_image']=blobs['map_image']
    cfg=dict(cfg);cfg['image']='map_image'
    saved['map_yaml']=yaml.safe_dump(cfg,sort_keys=True).encode()
    manifest={}
    for name,raw in saved.items():
        (folder/name).write_bytes(raw)
        manifest[name]={'source_sha256':hashlib.sha256(blobs[name]).hexdigest(),'snapshot_sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)}
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return folder,manifest
