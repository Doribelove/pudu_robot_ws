from pathlib import Path
import ast, csv, hashlib, json
root = Path(__file__).resolve().parent
manifest = json.loads((root / 'MANIFEST.json').read_text())
actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()
          and not {'__pycache__', '.pytest_cache'}.intersection(p.parts) and p.suffix != '.pyc'}
assert actual == set(manifest['files']) | {'MANIFEST.json'}
for rel, digest in manifest['files'].items():
    assert hashlib.sha256((root / rel).read_bytes()).hexdigest() == digest, rel
sources = json.loads((root / 'metadata/SOURCE_PROVENANCE.json').read_text())
for rel, item in sources.items():
    assert hashlib.sha256((root / rel).read_bytes()).hexdigest() == item['sha256'], rel
for folder in ['src', 'profiles/p6/src']:
    for path in (root / folder).rglob('*.py'):
        ast.parse(path.read_text())
for name, count in [('p6_local_requests.csv', 1066), ('p7_r1_formal_requests.csv', 224)]:
    with (root / 'report' / name).open() as stream:
        assert len(list(csv.DictReader(stream))) == count, name
assert manifest['branch'] == 'feature/liyongqi'
for rel in actual:
    assert Path(rel).suffix.lower() not in {'.png', '.pgm', '.jpg', '.npy', '.npz', '.bag', '.db3', '.mp4', '.gif'}, rel
print(f"Verified {len(actual)} files, {len(sources)} source records, and 1066 + 224 measured requests.")
