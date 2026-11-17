#!/usr/bin/env python3
"""Collect only this CI workspace's logs/provenance, never home or credentials."""
from pathlib import Path
import json,shutil,sys
work=Path(sys.argv[1]).resolve();out=work/'evidence';out.mkdir(exist_ok=True)
copied=[];skipped=[]
roots=[work/'log',work/'build',work/'install']
tmp=Path(sys.argv[3]).resolve() if len(sys.argv)>3 else None
if tmp is not None:
 if tmp.parent!=Path('/tmp') or not tmp.name.startswith('tri-ci-'):
  raise ValueError('Only this job-owned short temporary directory may be collected')
 roots.append(tmp)
for base in roots:
 if not base.exists():continue
 for p in base.rglob('*'):
  if not p.is_file() or p.is_symlink():continue
  relative=Path('test-tmp')/p.relative_to(tmp) if base==tmp else p.relative_to(work)
  if base==tmp:selected=(p.suffix in ('.log','.jsonl','.xml') or p.name in ('result.json','summary.json','complete.json','input_snapshot.json'))
  elif base.name=='log':selected=True
  elif base.name=='build':selected=(p.suffix in ('.log','.jsonl','.xml') or p.name in ('complete.json','CTestTestfile.cmake','CMakeCache.txt'))
  else:selected=p.name in ('BUILD_PROVENANCE.json','SOURCE_MANIFEST.json','SOURCES_AND_LICENSES.json')
  if not selected:continue
  if p.stat().st_size>32*1024*1024:skipped.append(str(relative));continue
  dest=out/'collected'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest);copied.append(str(relative))
(out/'completion.json').write_text(json.dumps({'exit_code':int(sys.argv[2]),'copied':copied,'over_32_MiB_skipped':skipped,'temporary_directory':str(tmp) if tmp else None,'temporary_directory_removed':False,'scope':'CI workspace and job-owned short temporary test logs/results plus source-build provenance only'},indent=2)+'\n')
