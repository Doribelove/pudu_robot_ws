#!/usr/bin/python3
"""Copy selected crates; generate fail-closed resolution-only stubs for others.
Original numerical code, features and lock are not changed. Input is local/read-only.
"""
import pathlib,hashlib,shutil,re,json,tomli,toml
b=pathlib.Path('/tmp/trilayer-clarabel-build'); d=pathlib.Path('/tmp/trilayer-clarabel-filtered')
keys=set(); platforms={}
for name, path in [('x86_64-unknown-linux-gnu',b/'ACTIVE_DEPENDENCIES.txt'),('aarch64-unknown-linux-gnu',d/'ACTIVE_AARCH64.txt')]:
 subset=set()
 for l in path.read_text().splitlines():
  m=re.match(r'(\S+) v(\S+)',l)
  if m and m[1]!='clarabel': subset.add(f'{m[1]}-{m[2]}')
 keys|=subset;platforms[name]=sorted(subset)
rows=[]
for src in sorted((b/'vendor').iterdir()):
 dst=d/'vendor'/src.name
 if src.name in keys:
  shutil.copytree(src,dst,dirs_exist_ok=True)
  rows.append({'crate':src.name,'status':'original_source','original_manifest_sha256':hashlib.sha256((src/'Cargo.toml').read_bytes()).hexdigest()});continue
 # Existing inactive dirs are owned by this script only.
 if dst.exists(): shutil.rmtree(dst)
 (dst/'src').mkdir(parents=True)
 original=tomli.loads((src/'Cargo.toml').read_text());p=original['package']
 # Keep only resolution facts; no upstream implementation, license text or original manifests.
 stub={k:original[k] for k in ['dependencies','build-dependencies','dev-dependencies','target','features'] if k in original}
 stub['package']={k:p[k] for k in ['name','version','edition','rust-version'] if k in p}
 stub['package'].update({'build':False,'autobins':False,'autoexamples':False,'autotests':False,'autobenches':False,'metadata':{'trilayer-filter':{'resolution-only-stub':True,'not-upstream-source':True}}})
 stub['lib']={'path':'src/lib.rs'}
 (dst/'Cargo.toml').write_text('# GENERATED resolution-only metadata; not the upstream crate. Never compile.\n'+toml.dumps(stub))
 (dst/'src/lib.rs').write_text('compile_error!("Excluded dependency: this is a resolution-only stub, not upstream source. Regenerate vendor for the requested feature/target.");\n')
 checksum=json.loads((src/'.cargo-checksum.json').read_text())
 checksum['files']={str(f.relative_to(dst)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [dst/'Cargo.toml',dst/'src/lib.rs']}
 (dst/'.cargo-checksum.json').write_text(json.dumps(checksum,sort_keys=True))
 rows.append({'crate':src.name,'status':'generated_resolution_stub','original_manifest_sha256':hashlib.sha256((src/'Cargo.toml').read_bytes()).hexdigest(),'generated_manifest_sha256':hashlib.sha256((dst/'Cargo.toml').read_bytes()).hexdigest()})
(d/'FILTER_MANIFEST.json').write_text(json.dumps({'method':'selected original source + generated fail-closed resolution-only stubs (no upstream code in stubs)','platform_sets':platforms,'source_crates':len(keys),'resolution_stubs':len(rows)-len(keys),'rows':rows,'arm_build_status':'not tested','reference':'https://github.com/coreos/cargo-vendor-filterer (same resolution-stub principle; this is independent implementation, not execution of that tool)'},indent=2)+'\n')
print('source',len(keys),'stubs',len(rows)-len(keys),'arm additional',len(set(platforms['aarch64-unknown-linux-gnu'])-set(platforms['x86_64-unknown-linux-gnu'])))
