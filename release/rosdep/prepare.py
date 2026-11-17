from pathlib import Path
import subprocess,re
root=Path(__file__).parent;repo=root/'rosdistro'
def insert(file,key,block):
 s=file.read_text();keys=[(m.group(1),m.start()) for m in re.finditer(r'^([^ #\n][^:\n]*):',s,re.M)]
 assert key not in [k for k,_ in keys]
 pos=next((p for k,p in keys if k>key),len(s));file.write_text(s[:pos]+block+s[pos:])
for key in ['python3-pybind11','python3-setuptools-scm']:
 insert(repo/'rosdep/python.yaml',key,f'{key}:\n  debian: [{key}]\n  ubuntu: [{key}]\n')
(root/'01-python-build-dependencies.patch').write_bytes(subprocess.check_output(['git','diff','--','rosdep/python.yaml'],cwd=repo))
for key in ['cargo-1.85','rustc-1.85']:
 insert(repo/'rosdep/base.yaml',key,f'{key}:\n  ubuntu:\n    jammy: [{key}]\n    noble: [{key}]\n')
(root/'02-optional-pinned-rust-1.85.patch').write_bytes(subprocess.check_output(['git','diff','--','rosdep/base.yaml'],cwd=repo))
