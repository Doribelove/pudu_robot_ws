#!/usr/bin/python3
"""Bounded diagnostics only; no binaries/rootfs/credentials, no build execution."""
import json,sys,shutil
from pathlib import Path
out=Path(sys.argv[1]).resolve();tmp=Path(sys.argv[2]).resolve()
dest=out/'evidence'/'diagnostics';dest.mkdir(parents=True,exist_ok=True)
rows=[];total=0;cap=64*1024*1024
for label,root in [('build',out/'build'),('temporary',tmp)]:
 for path in sorted(root.rglob('*')):
  if path.is_symlink() or not path.is_file():continue
  rel=path.relative_to(root)
  # CTest/CMake/ament logs plus smoke JSON/YAML; avoid vendored source bulk.
  keep=(label=='temporary' and path.suffix in {'.log','.json','.txt','.yaml'}) or (label=='build' and (path.name in {'CMakeCache.txt','CMakeError.log','CMakeOutput.log','LastTest.log','LastTestsFailed.log','CTestCostData.txt','control','rules','changelog'} or 'test_results' in path.parts or ('Testing' in path.parts and path.suffix in {'.log','.txt','.xml'})))
  if not keep:continue
  n=path.stat().st_size;copied=n<=2*1024*1024 and total+n<=cap
  rows.append({'root':label,'path':str(rel),'bytes':n,'copied':copied})
  if copied:
   target=dest/label/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,target);total+=n
(dest/'MANIFEST.json').write_text(json.dumps({'bytes_copied':total,'cap_bytes':cap,'files':rows},indent=2)+'\n')
