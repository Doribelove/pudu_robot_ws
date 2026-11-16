"""Create a runnable copy of the fixed company package; preserve every published byte."""
from pathlib import Path
import argparse, hashlib, json, shutil
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3ydl3_v13_r0_astar_compare_20261023')
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=ROOT/'source/3YDL3-V13-L2');p.add_argument('--runtime',type=Path,default=ROOT/'runtime/3YDL3-V13-L2');p.add_argument('--complete-assets',action='store_true');a=p.parse_args()
 assert not a.runtime.exists() or a.complete_assets, 'New runtime directory required'
 manifest=json.loads((a.source/'MANIFEST.json').read_text());assert manifest['default_profile']=='p6'
 for rel,digest in manifest['files'].items(): assert sha(a.source/rel)==digest, rel
 if not a.runtime.exists():shutil.copytree(a.source,a.runtime)
 additions={}
 for rel in ['config/l3.json','config/endpoint_tolerance.json','config/regional_mixed_mengjia.json','native/curve_nav2_stack','tools/curve_composed.launch.py','tools/curve_stack.launch.py']:
  src=PARENT/rel;dst=a.runtime/'base/runtime/base'/rel
  assert src.is_file(),src
  if dst.exists():assert sha(src)==sha(dst),dst
  dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
  additions[str(dst.relative_to(a.runtime))]={'source':str(src),'sha256':sha(src),'reason':'unchanged inherited P6 runtime asset omitted from company source package'}
 original=Path('/home/robot/workspaces/3yd_v1_semantic_ack_20260915')
 eval_root=original/'external/arena4_ws/src/arena/evaluation/arena_evaluation'
 extra=[(original/'build/libsemantic_sweep.so','resources/semantic/build/libsemantic_sweep.so'),(original/'config/fastdds_large_map.xml','resources/semantic/config/fastdds_large_map.xml')]
 extra += [(p,'base/runtime/config/'+p.name) for p in sorted((eval_root/'config').glob('planner_benchmark*.yaml'))]
 for src,rel in extra:
  dst=a.runtime/rel;assert src.is_file(),src
  if dst.exists():assert sha(src)==sha(dst),dst
  dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
  additions[rel]={'source':str(src),'sha256':sha(src),'reason':'unchanged semantic sweep/transport/benchmark resource from original P6 inherited runtime'}
 for rel,digest in manifest['files'].items(): assert sha(a.runtime/rel)==digest,rel
 data={'source':str(a.source.resolve()),'runtime':str(a.runtime.resolve()),'fixed_commit':'d8c01c5a71a67c2e85c250e8a8148bda9724dc68','fixed_manifest_sha256':sha(a.source/'MANIFEST.json'),'published_files_unchanged':len(manifest['files'])+1,'additions':additions,'solver_changes':False,'parameter_changes':False}
 (ROOT/'docs/RUNTIME_ASSETS.json').write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(data,indent=2))
if __name__=='__main__':main()
