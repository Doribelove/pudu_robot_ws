"""Fresh canonical semantic hash without copying a known primitive schema."""
from dataclasses import fields
import hashlib,inspect,json
from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature,ConversionDiagnostic

_to_dict=SemanticMapV1.to_dict
_getter=SemanticMapV1.semantic_map_hash.fget
_source_matches=(hashlib.sha256(inspect.getsource(_to_dict).encode()).hexdigest()==
    '01e0cd3d4111f116b08b9a898a4ac2f93055b38e01c9ecc7bdf200066fc6cb3c'
    and hashlib.sha256(inspect.getsource(_getter).encode()).hexdigest()==
    'b614753287817a71272bdb3918f498ec2beac913050cd4cbbdb960f449c46c5f')
_counts={'fresh_fast_hashes':0,'parent_hashes':0}

def plain_tree(value):
    stack=[value];seen=set()
    while stack:
        v=stack.pop();kind=type(v)
        if v is None or kind in (str,bool,int,float):continue
        if kind in (dict,list,tuple):
            identity=id(v)
            if identity in seen:return False # aliases/cycles retain asdict's exact parent behavior
            seen.add(identity)
            if kind is dict:stack.extend(v.keys());stack.extend(v.values())
            else:stack.extend(v)
        elif kind in (SemanticFeature,ConversionDiagnostic):
            identity=id(v)
            if identity in seen:return False
            seen.add(identity)
            stack.extend(getattr(v,f.name) for f in fields(v))
        else:return False
    return True

def phase_hash(semantic):
    if (type(semantic) is not SemanticMapV1 or not _source_matches
        or SemanticMapV1.to_dict is not _to_dict or getattr(SemanticMapV1.semantic_map_hash,'fget',None) is not _getter):
        return None
    value={key:getattr(semantic,key) for key in ('schema_version','frame_id','resolution','width','height',
        'source_pdmap_hash','features','traffic_rules','diagnostics','unrecognized_fields')}
    value['origin']=list(semantic.origin)
    if not plain_tree(value):
        return None
    def dataclass_values(v):
        if type(v) in (SemanticFeature,ConversionDiagnostic):return {f.name:getattr(v,f.name) for f in fields(v)}
        raise TypeError('SEMANTIC_SCHEMA_CHANGED_DURING_HASH')
    payload=json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False,
        default=dataclass_values).encode('utf-8')
    _counts['fresh_fast_hashes']+=1
    return hashlib.sha256(payload).hexdigest()

def fresh_hash(semantic):
    value=phase_hash(semantic)
    if value is not None:return value
    _counts['parent_hashes']+=1;return semantic.semantic_map_hash

def stats():return {**_counts,'semantic_content_cached':False,'unknown_schema_uses_original_hash':True}
