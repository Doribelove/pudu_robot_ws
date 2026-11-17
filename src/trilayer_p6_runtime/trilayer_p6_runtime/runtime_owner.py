"""Shared owned initialization/cleanup for one-shot and persistent public APIs."""
from pathlib import Path
import hashlib,sys,traceback
from .api import SemanticRequest,validate_request,validate_semantic_request
from . import RuntimeConfig,activate

class RuntimeOwner:
    def __init__(self,request,*,resource_root=None,solver_root=None):
        self.request=request;self.semantic=isinstance(request,SemanticRequest)
        self.resource_root=resource_root;self.solver_root=solver_root
        self.session=None;self.service=None;self.world=None;self.activation={};self.manifest=None
        self.out=Path(request.output_dir);self.solver_sources={};self.closed=False;self.close_result=None
    def open(self):
        if self.closed or self.session is not None:raise RuntimeError('Runtime owner cannot be reopened')
        try:_initialize(self)
        except BaseException:
            self.close()
            raise
        return self
    def integrity_errors(self):
        errors=[]
        for name,record in (self.manifest or {}).items():
            try:
                if hashlib.sha256((self.out/'inputs'/name).read_bytes()).hexdigest()!=record['snapshot_sha256']:
                    errors.append('Input snapshot changed during execution: '+name)
            except OSError as exc:errors.append('Input snapshot unavailable: '+name+': '+str(exc))
        return errors
    def close(self):
        if self.closed:return self.close_result
        errors=[]
        for label,obj in [('service',self.service),('session',self.session)]:
            if obj is not None:
                try:obj.close()
                except BaseException:errors.append(label+': '+traceback.format_exc())
        errors.extend(self.integrity_errors())
        code=getattr(getattr(getattr(self.session,'stack',None),'process',None),'returncode',None)
        self.close_result={'closed':True,'cleanup_errors':errors,'worker_exit':code}
        self.closed=True
        return self.close_result

def _initialize(self):
    request=self.request;resource_root=self.resource_root;solver_root=self.solver_root;semantic=self.semantic
    mp,out=validate_request(request)
    semantic_inputs=validate_semantic_request(request) if semantic else None
    map_inputs=None
    if not semantic:
        import yaml
        raw=mp.read_bytes();cfg=yaml.safe_load(raw);image=Path(cfg['image']);image=image if image.is_absolute() else mp.parent/image
        image_raw=image.read_bytes()
        validate_request(request)
        if mp.read_bytes()!=raw or image.read_bytes()!=image_raw:raise ValueError('Map changed during snapshot validation')
        map_inputs=({'map_yaml':raw,'map_image':image_raw},cfg)
    if solver_root is not None:
        solver_root=Path(solver_root).resolve()
        if not solver_root.is_dir():raise ValueError('solver_root must be an existing directory')
        sys.path.insert(0,str(solver_root))
    import osqp,qdldl,clarabel
    solver_sources={n:str(Path(m.__file__).resolve()) for n,m in [('osqp',osqp),('qdldl',qdldl),('clarabel',clarabel)]}
    if solver_root is not None and not all(Path(f).is_relative_to(solver_root) for f in solver_sources.values()):
        raise ValueError('Solvers already imported outside explicit solver_root')
    self.out=out;self.solver_sources=solver_sources
    out.mkdir(parents=True,exist_ok=False)
    self.activation=activate(RuntimeConfig(request.master_service,out/'cache'),resource_root=resource_root)
    from arena_evaluation import unified_four_backends_smoke as rt
    from three_yd_v5.free_space_topology import load_map_only_world,FreeSpaceTopology
    from three_yd_v5.v10_storage import prepare
    from three_yd_v5.curve_session import CurveSession
    from three_yd_v5.v12_topology import persistent
    from three_yd_v5.v12_fields import prepare_fields
    from arena_3d_v1.multires_grid import GridView
    from three_yd_v5.v9_gate_init import DirectStorageAuditor
    from three_yd_v13.service import IncrementalService
    from arena_evaluation.planner_benchmark.models import Query
    if semantic:
        from .semantic_inputs import snapshot_inputs
        folder,self.manifest=snapshot_inputs(out,*semantic_inputs)
        mp=folder/'map_yaml'
        from arena_3d_v1.semantic_world import SemanticWorld
        from three_yd_v5.v10_firstready import firstready_construction
        from three_yd_v5.v10_storage import prepare_topology
        from three_yd_v5.topology_default import DefaultTopology
        with firstready_construction():
            world=SemanticWorld(mp,folder/'semantic.json',cache_root=out/'world');prepare(world)
    else:
        from .semantic_inputs import snapshot_inputs
        folder,self.manifest=snapshot_inputs(out,*map_inputs)
        mp=folder/'map_yaml'
        world=load_map_only_world(mp,out/'world');prepare(world)
    ctx=rt.MapContext('request_map',world.map,world.safe,world.map.distance_m,world.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
    session=CurveSession(ctx,out/'session',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
    self.session=session;self.world=world
    session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
    session.start()
    if semantic:
        graph=persistent(world,DefaultTopology,out/'cache/indices/request_map',folder/'topology.json');prepare_topology(graph)
    else:
        graph=persistent(world,FreeSpaceTopology,out/'cache/indices/request_map',None)
    prepare_fields(world,graph,out/'cache/planes')
    views={3:GridView(world,3),1:GridView(world,1)}
    auditor=DirectStorageAuditor(ctx,source_commit='P6-source-port-api')
    service=IncrementalService(world,graph,out/'l2_cache',views,session,auditor,rt.backend_availability()['hybrid_astar'],arm='H-I',max_region_routes=3 if semantic else 1)
    self.service=service
