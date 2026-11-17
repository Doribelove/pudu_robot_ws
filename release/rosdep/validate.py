#!/usr/bin/python3
"""Validate proposal locally; never touches global rosdep or apt state."""
from pathlib import Path
import sys,subprocess,json,yaml,hashlib
from rosdep2.lookup import RosdepDefinition
from rosdep2.platforms.debian import AptInstaller
r=Path(__file__).parent;repo=r/'rosdistro';keys=['python3-setuptools-scm','python3-pybind11','rustc-1.85','cargo-1.85'];found={k:[] for k in keys}
for f in (repo/'rosdep').glob('*.yaml'):
 d=yaml.safe_load(f.read_text()) or {}
 for key in keys:
  if key in d:found[key].append((str(f.relative_to(repo)),d[key]))
evidence=json.loads((r/'OFFICIAL_PACKAGE_EVIDENCE.json').read_text())
for item in evidence:
 assert hashlib.sha256((r/item['saved_file']).read_bytes()).hexdigest()==item['sha256']
 assert item['covers_amd64_arm64']
results={}
for key,rows in found.items():
 assert len(rows)==1,(key,rows)
 path,data=rows[0];rd=RosdepDefinition(key,data,path)
 installer,rule=rd.get_rule_for_platform('ubuntu','jammy',['apt'],'apt')
 assert installer=='apt';packages=AptInstaller().resolve(rule);assert packages==[key],(key,packages)
 results[key]={'file':path,'jammy_installer':installer,'packages':packages}
for f in ['rosdep/python.yaml','rosdep/base.yaml']:
 subprocess.run([sys.executable,'scripts/check_rosdep.py',f],cwd=repo,check=True)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
# Patches must reverse-apply against exactly this proposed working tree.
for f in ['01-python-build-dependencies.patch','02-optional-pinned-rust-1.85.patch']:
 subprocess.run(['git','apply','--reverse','--check',str(r/f)],cwd=repo,check=True)
result={'upstream_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),'result':'PASS','resolution':results,'limitations':'No package install or full upstream CI/buildfarm run. ARM availability is evidence only, not compilation. Existing cross-file keys other than proposed additions not audited.'}
(r/'VALIDATION.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
