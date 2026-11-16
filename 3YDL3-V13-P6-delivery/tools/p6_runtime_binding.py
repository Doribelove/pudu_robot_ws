"""Read-only provenance audit for the fixed P6 runtime and actual service classes."""
from pathlib import Path
import hashlib, importlib, json, os, sys
ROOT=Path(__file__).resolve().parents[1]
FAMILIES=('three_yd_v3','three_yd_v4','three_yd_v5','three_yd_v13','arena_3d_v1','arena_evaluation')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def package():return Path(os.environ['V13_CLOSEOUT_PACKAGE']).resolve()
def expected_files():
 p=package();d=json.loads((p/'MANIFEST.json').read_text())['files'];d=dict(d)
 for rel,item in json.loads((ROOT/'docs/RUNTIME_ASSETS.json').read_text())['additions'].items():d[rel]=item['sha256']
 return d
def audit(output=None):
 p=package();expected=expected_files();records={};outside=[]
 for name,module in sorted(sys.modules.items()):
  path=getattr(module,'__file__',None)
  if not path or name.split('.')[0] not in FAMILIES:continue
  f=Path(path).resolve()
  if not f.is_relative_to(p):outside.append({'module':name,'path':str(f)});continue
  rel=str(f.relative_to(p));digest=sha(f)
  assert rel in expected and expected[rel]==digest,(name,rel,'UNREGISTERED_OR_CHANGED_SOURCE')
  records[name]={'path':str(f),'sha256':digest,'published_source':rel in json.loads((p/'MANIFEST.json').read_text())['files']}
 assert not outside,('ALGORITHM_MODULE_OUTSIDE_FIXED_RUNTIME',outside)
 from three_yd_v5.curve_backend import settings,CONFIG_PATH
 from three_yd_v5.endpoint_tolerance import policy
 mode,cfg,digest=settings();tol=policy()
 assert mode=='independent',mode
 assert cfg['total_request_budget_s']==30,cfg['total_request_budget_s']
 # The inherited field name is recorded whole; no settings are overwritten here.
 data={'version':'3YDL3-V13-P6','profile':'p6','arm':'H-I','fixed_commit':'d8c01c5a71a67c2e85c250e8a8148bda9724dc68','runtime':str(p),'loaded_algorithm_modules':records,'outside_algorithm_modules':outside,'l3_config_path':str(CONFIG_PATH),'l3_config_sha256':sha(CONFIG_PATH),'effective_l3_config':cfg,'effective_l3_config_sha256':digest,'endpoint_tolerance_policy':tol,'resource_bindings':RESOURCE_BINDINGS,'environment_binding':{k:os.environ.get(k) for k in ['THREEYD_BENCH_SOURCE','THREEYD_V12_INDEX_ROOT','THREEYD_V12_PLANE_ROOT']},'read_only_audit_outside_request_timing':True}
 if output:Path(output).write_text(json.dumps(data,indent=2,default=str)+'\n')
 return data
def service_factory(*args,**kwargs):
 from three_yd_v13.service import IncrementalService
 kwargs.pop('continuation',None)
 return IncrementalService(*args,arm='H-I',**kwargs)
def identity(service):
 c=service.controller;state=getattr(getattr(c,'l2',None),'state',None)
 def cls(x):return None if x is None else type(x).__module__+'.'+type(x).__qualname__
 return {'service_class':cls(service),'arm':service.arm,'controller_class':cls(c),'L2_planner_class':cls(getattr(c,'l2',None)),'L2_state_class':cls(state),'dynamic_version':service.dynamic.version,'event_repair_requested':False,'entry':'IncrementalService.plan -> inherited TaskProgressService plan wrappers -> IncrementalService._solve/_p4_solve -> V5Runner(controller_class=IncrementalController)'}
def check():
 for name in ['three_yd_v13.service','three_yd_v13.schedule','three_yd_v13.incremental','three_yd_v13.state','three_yd_v5.curve_session','three_yd_v5.curve_backend','three_yd_v5.endpoint_tolerance','three_yd_v5.recovery','three_yd_v5.route_quality','three_yd_v5.p6_hint']:
  importlib.import_module(name)
 data=audit();print(json.dumps(data,indent=2,default=str));return data
def bind_resources():
 import arena_3d_v1.semantic_world as world
 import arena_3d_v1.semantic_pipeline as pipeline
 target=package()/'resources/semantic'
 assert (target/'build/libsemantic_sweep.so').is_file()
 records={}
 for module in (world,pipeline):
  records[module.__name__+'.WORK']={'original_layout_value':str(module.WORK),'bound_value':str(target),'kind':'resource directory only; solver functions and parameters unchanged'}
  module.WORK=target
 return records
RESOURCE_BINDINGS=bind_resources()
if __name__=='__main__':check()
