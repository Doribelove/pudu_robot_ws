#!/usr/bin/python3
"""Offline build into a private build directory, never install/download a toolchain."""
import argparse,hashlib,json,os,pathlib,platform,re,shutil,subprocess,sys,tarfile
p=argparse.ArgumentParser();p.add_argument('--source-root',type=pathlib.Path,required=True);p.add_argument('--work',type=pathlib.Path,required=True);p.add_argument('--cargo',required=True);p.add_argument('--rustc',required=True);p.add_argument('--jobs',type=int,default=2);p.add_argument('--toolchain-library-path',default='');a=p.parse_args()
def version(executable,label):
 version_env=os.environ.copy()
 if a.toolchain_library_path:version_env['LD_LIBRARY_PATH']=a.toolchain_library_path
 line=subprocess.check_output([executable,'--version'],text=True,env=version_env).strip();m=re.match(label+r' (\d+)\.(\d+)\.(\d+)',line)
 if not m or tuple(map(int,m.groups()))<(1,84,0):raise RuntimeError(f'{label} >=1.84 required; got {line}. Stock Jammy toolchain insufficient; supply audited toolchain paths. No automatic download.')
 return line
cargo=str(pathlib.Path(a.cargo).resolve());rustc=str(pathlib.Path(a.rustc).resolve());versions={'cargo':version(cargo,'cargo'),'rustc':version(rustc,'rustc')}
if platform.system()!='Linux' or platform.machine() not in ['x86_64','aarch64']:raise RuntimeError('Only Python/Linux x86_64 and aarch64 source closures are retained. ARM execution is unvalidated.')
if a.jobs<1:raise ValueError('jobs must be positive')
manifest=json.loads((a.source_root/'SOURCE_MANIFEST.json').read_text());archive=a.source_root/'sources'/manifest['source_archive']
if hashlib.sha256(archive.read_bytes()).hexdigest()!=manifest['sha256']:raise RuntimeError('source SHA256 mismatch')
work=a.work.resolve();marker=work/'.trilayer_clarabel_owned'
if work.exists() and any(work.iterdir()) and not marker.exists():raise RuntimeError('Refusing unowned nonempty work directory')
if work.exists():shutil.rmtree(work)
work.mkdir(parents=True);marker.touch()
with tarfile.open(archive) as t:
 for m in t.getmembers():
  q=pathlib.PurePosixPath(m.name)
  if q.is_absolute() or '..' in q.parts or not q.parts or q.parts[0] not in ['source','vendor'] or not m.isfile():raise RuntimeError('Unsafe source archive entry: '+m.name)
 t.extractall(work)
source=work/'source';(source/'.cargo').mkdir()
(source/'.cargo/config.toml').write_text('[source.crates-io]\nreplace-with="vendored-sources"\n[source.vendored-sources]\ndirectory='+json.dumps(str(work/'vendor'))+'\n[net]\noffline=true\n')
if hashlib.sha256((source/'Cargo.lock').read_bytes()).hexdigest()!=manifest['cargo_lock_sha256']:raise RuntimeError('lock mismatch')
env={'HOME':str(work),'PATH':str(pathlib.Path(cargo).parent)+':'+str(pathlib.Path(rustc).parent)+':/usr/bin:/bin','LANG':'C.UTF-8','CARGO_HOME':str(work/'cargo-home'),'CARGO_TARGET_DIR':str(work/'target'),'RUSTC':rustc,'PYO3_PYTHON':sys.executable,'PYTHONNOUSERSITE':'1','CARGO_NET_OFFLINE':'true'}
if a.toolchain_library_path:env['LD_LIBRARY_PATH']=a.toolchain_library_path
cmd=[cargo,'build','--locked','--offline','--release','--features','python','--message-format=json-render-diagnostics','-j',str(a.jobs)]
with (work/'build.jsonl').open('w') as stdout,(work/'build.log').open('w') as stderr:
 result=subprocess.run(cmd,cwd=source,env=env,stdout=stdout,stderr=stderr)
if result.returncode:
 print((work/'build.log').read_text()[-12000:],file=sys.stderr);raise RuntimeError('Offline Cargo build failed; full log in '+str(work/'build.log'))
compiled=set()
for line in (work/'build.jsonl').read_text().splitlines():
 try:r=json.loads(line)
 except ValueError:continue
 if r.get('reason')=='compiler-artifact':
  path=pathlib.Path(r['manifest_path'])
  if path.parent.parent==work/'vendor':compiled.add(path.parent.name)
filters=json.loads((a.source_root/'provenance/FILTER_MANIFEST.json').read_text());stubs={r['crate'] for r in filters['rows'] if r['status']=='generated_resolution_stub'}
if compiled&stubs:raise RuntimeError('Unexpected stub compilation')
pydir=work/'python/clarabel';pydir.mkdir(parents=True)
shutil.copy2(source/'python/clarabel/__init__.py',pydir/'__init__.py');shutil.copy2(work/'target/release/libclarabel.so',pydir/'clarabel.so')
proof={'source_manifest':manifest,'toolchain':versions,'toolchain_paths':{'cargo':cargo,'rustc':rustc,'explicit_library_path':a.toolchain_library_path},'python':sys.version,'python_executable':sys.executable,'architecture':platform.machine(),'ARM_validation':'not performed','command':cmd,'compiled_crates':sorted(compiled),'compiled_stubs':[],'extension_sha256':hashlib.sha256((pydir/'clarabel.so').read_bytes()).hexdigest(),'features':'Original python + defaults (faer/sdp retained)','release_status':'NOT RELEASED; see source manifest blockers'}
(work/'complete.json').write_text(json.dumps(proof,indent=2)+'\n')
