"""Real, single-request P6 map-only adapter; invoke in an isolated bounded process.

Does not publish velocity commands. Uses fixed original P6 vehicle/algorithm
contract. Callers MUST reserve a separate DDS domain and enforce a wall timeout.
"""
from dataclasses import dataclass,asdict,is_dataclass
from pathlib import Path
import hashlib,json,math,os,time,traceback,sys
from . import RuntimeConfig,activate

@dataclass(frozen=True)
class MapOnlyRequest:
    map_yaml: Path
    start: tuple
    goal: tuple
    output_dir: Path
    master_service: str='/global_costmap/trilayer_get_costmap'
    vehicle: tuple=(.265,.225,.40)
    query_id: str='request'
    seed: int=20260929

@dataclass(frozen=True,kw_only=True)
class SemanticRequest(MapOnlyRequest):
    semantic_json: Path
    topology_json: Path
    parking_access_json: Path

def validate_semantic_request(request):
    if not isinstance(request,SemanticRequest):raise TypeError('SemanticRequest required')
    # Detect edits between base-map validation and retaining snapshot bytes.
    import yaml
    mp=Path(request.map_yaml);raw=mp.read_bytes();cfg=yaml.safe_load(raw)
    if not isinstance(cfg,dict) or not isinstance(cfg.get('image'),str):
        validate_request(request)  # raises the public descriptive error
    image=Path(cfg['image']);image=image if image.is_absolute() else mp.parent/image
    image_digest=hashlib.sha256(image.read_bytes()).hexdigest()
    validate_request(request)
    from .semantic_inputs import read_inputs
    blobs,cfg=read_inputs(request)
    if blobs['map_yaml']!=raw or hashlib.sha256(blobs['map_image']).hexdigest()!=image_digest:
        raise ValueError('Map changed during semantic input validation; retry with stable inputs')
    return blobs,cfg

def _json(value):
    if is_dataclass(value):return _json(asdict(value))
    if isinstance(value,dict):return {str(k):_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [_json(v) for v in value]
    if hasattr(value,'tolist'):return _json(value.tolist())
    if isinstance(value,Path):return str(value)
    if value is None or isinstance(value,(str,int,float,bool)):return value
    raise TypeError('Unsupported runtime output type: '+type(value).__name__)

def validate_request(request):
    """Validate supported static-map contract before starting ROS processes."""
    import yaml
    from PIL import Image
    if not isinstance(request,MapOnlyRequest):raise TypeError('MapOnlyRequest required')
    mp=Path(request.map_yaml)
    if not mp.is_absolute() or not mp.is_file():raise ValueError('map_yaml must be an existing absolute path')
    if tuple(request.vehicle)!=(.265,.225,.40):raise ValueError('P6 currently supports only vehicle (.265, .225, .40); dimensions/radius are not configurable')
    for label,pose in [('start',request.start),('goal',request.goal)]:
        if len(pose)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in pose):
            raise ValueError(label+' must contain three finite numeric x, y, yaw values')
    out=Path(request.output_dir)
    if not out.is_absolute() or out.exists():raise ValueError('output_dir must be a new absolute directory')
    if not request.query_id or not isinstance(request.query_id,str):raise ValueError('query_id must be nonempty text')
    if not isinstance(request.seed,int) or isinstance(request.seed,bool):raise ValueError('seed must be an integer')
    if request.master_service!='/global_costmap/trilayer_get_costmap':raise ValueError('This native worker currently owns /global_costmap/trilayer_get_costmap only')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('ROS_LOCALHOST_ONLY=1 is required')
    try:domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    except ValueError:domain=0
    if not 1<=domain<=232:raise ValueError('Reserve and explicitly set a nondefault ROS_DOMAIN_ID in 1..232')
    try:cfg=yaml.safe_load(mp.read_text())
    except (OSError,UnicodeError,yaml.YAMLError) as exc:
        raise ValueError('Map YAML cannot be read or parsed: '+str(exc)) from exc
    if not isinstance(cfg,dict):raise ValueError('Map YAML must be a mapping')
    numeric=lambda v: isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
    resolution=cfg.get('resolution')
    if not numeric(resolution) or resolution!=.05:
        raise ValueError('Map resolution must be numeric 0.05 m, not a string or boolean')
    origin=cfg.get('origin')
    if not isinstance(origin,(list,tuple)) or len(origin)!=3 or not all(numeric(v) for v in origin) or origin[2]!=0:
        raise ValueError('Map origin must contain finite numeric x,y and zero rotation, not strings or booleans')
    if cfg.get('mode','trinary')!='trinary':raise ValueError('Only trinary occupancy maps are supported')
    negate=cfg.get('negate',0)
    if type(negate) is not int or negate not in (0,1):
        raise ValueError('Map negate must be integer 0 or 1, not a string or boolean')
    free=cfg.get('free_thresh',.196);occupied=cfg.get('occupied_thresh',.65)
    if not numeric(free) or not numeric(occupied) or not 0<=free<occupied<=1:
        raise ValueError('Map thresholds must be finite numbers satisfying 0 <= free_thresh < occupied_thresh <= 1')
    image_value=cfg.get('image')
    if not isinstance(image_value,str) or not image_value.strip() or '\x00' in image_value:
        raise ValueError('Map image must be a nonempty string path')
    image=Path(image_value);image=image if image.is_absolute() else mp.parent/image
    try:
        with Image.open(image) as im:
            width,height=im.size
            if width<16 or height<16 or width*height>1000000:
                raise ValueError('Map must be at least 16x16 and at most 1000000 cells in this bounded adapter')
            im.verify()
        # Header/verify alone is insufficient for formats with truncated pixel data.
        with Image.open(image) as im:im.load();im.convert('L').load()
    except (OSError,SyntaxError,ValueError,Image.DecompressionBombError) as exc:
        raise ValueError('Map image is missing, corrupt, or unsupported: '+str(exc)) from exc
    for label,pose in [('start',request.start),('goal',request.goal)]:
        if not (origin[0]<=pose[0]<origin[0]+width*.05 and origin[1]<=pose[1]<origin[1]+height*.05):raise ValueError(label+' lies outside map bounds')
    return mp,out

def _plan(request,*,resource_root=None,solver_root=None,semantic=False):
    """Run once through the same owned initialization as persistent sessions."""
    # Preserve pre-launch validation errors and their no-output side effect.
    validate_request(request)
    if semantic:validate_semantic_request(request)
    from .runtime_owner import RuntimeOwner
    owner=RuntimeOwner(request,resource_root=resource_root,solver_root=solver_root)
    result={};failure=None;path=[];planned=None
    try:
        owner.open()
        from arena_evaluation.planner_benchmark.models import Query
        q=Query(request.query_id,tuple(request.start),tuple(request.goal),seed=request.seed)
        owner.session.reset_query_state(request.query_id,restore_base_map=False)
        start=time.monotonic();result=owner.service.plan(q);result['wall_s']=time.monotonic()-start
        planned=result.pop('result',None)
        if planned is not None:path=[[float(p['x']),float(p['y']),float(p['yaw'])] for p in (planned.points or [])]
    except BaseException:failure=traceback.format_exc()
    finally:closed=owner.close()
    final_success=bool(result.get('success')) and failure is None and not closed['cleanup_errors'] and closed['worker_exit']==0
    if not final_success:path=[]
    data=_json({'schema':'trilayer-p6-semantic-v1' if semantic else 'trilayer-p6-map-only-v1','mode':'semantic' if semantic else 'map_only','input_snapshot':owner.manifest,'frame_id':'map','solver_sources':owner.solver_sources,'plan_result':planned,'request':asdict(request),'success':final_success,'path':path,'result':result,'exception':failure,'cleanup_errors':closed['cleanup_errors'],'worker_exit':closed['worker_exit'],'ack_events':getattr(owner.session,'ack_events',[]),'l3_config_hash':owner.activation.get('l3_config_hash')})
    if owner.out.is_dir():(owner.out/'result.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    return data


def plan_map(request,*,resource_root=None,solver_root=None):
    """Execute one map-only query; see MapOnlyRequest and validate_request."""
    if isinstance(request,SemanticRequest):raise TypeError('Use plan_semantic for SemanticRequest')
    return _plan(request,resource_root=resource_root,solver_root=solver_root)


def plan_semantic(request,*,resource_root=None,solver_root=None):
    """Execute the strict static semantic subset from an immutable input snapshot.

    A fresh interpreter, dedicated localhost DDS domain and caller-enforced 60s
    bound are required. Unknown rules/fields are rejected, never silently ignored.
    """
    if not isinstance(request,SemanticRequest):raise TypeError('SemanticRequest required')
    return _plan(request,resource_root=resource_root,solver_root=solver_root,semantic=True)
