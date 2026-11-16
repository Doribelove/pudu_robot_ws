from pathlib import Path
import ast, csv, hashlib, json
root = Path(__file__).resolve().parent
manifest = json.loads((root / 'MANIFEST.json').read_text())
actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()
          and not {'__pycache__', '.pytest_cache'}.intersection(p.parts) and p.suffix != '.pyc'}
assert actual == set(manifest['files']) | {'MANIFEST.json'}
for rel, h in manifest['files'].items():
    assert hashlib.sha256((root/rel).read_bytes()).hexdigest() == h, rel
lock = json.loads((root/'SOURCE_LOCK.json').read_text())
assert lock['commit'] == 'd8c01c5a71a67c2e85c250e8a8148bda9724dc68'
assert lock['profile'] == 'p6' and lock['l2_arm'] == 'H-I'
for rel, h in lock['files'].items():
    for folder in ['source/3YDL3-V13-L2', 'runtime/3YDL3-V13-L2']:
        assert hashlib.sha256((root/folder/rel).read_bytes()).hexdigest() == h, rel
for rel, item in json.loads((root/'docs/RUNTIME_ASSETS.json').read_text())['additions'].items():
    assert hashlib.sha256((root/'runtime/3YDL3-V13-L2'/rel).read_bytes()).hexdigest() == item['sha256'], rel
for path, n in [('report/static300/p6_static_requests.csv', 300), ('report/ablation400/requests.csv', 400)]:
    with (root/path).open() as f:
        assert len(list(csv.DictReader(f))) == n, path
for path in (root/'tools').glob('*.py'):
    ast.parse(path.read_text())
assert manifest['branch'] == 'feature/liyongqi'
print(f"Verified {len(actual)} delivery files; fixed P6 source449 + runtime16; 300 regression and400 design records.")
