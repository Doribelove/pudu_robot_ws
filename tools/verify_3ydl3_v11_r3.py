from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'research/3ydl3_v11_r3/metadata/PUBLICATION_MANIFEST.json').read_text())
for relative, expected in manifest['files'].items():
    path = root / relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise SystemExit('MISSING_OR_CHANGED: ' + relative)
project = root / 'research/3ydl3_v11_r3'
for name in ('inputs', 'datasets', 'MAPS.json', 'results'):
    if (project / name).exists():
        raise SystemExit('EXCLUDED_MAP_OR_RUNTIME_DATA_PRESENT: ' + name)
print('Verified', len(manifest['files']), 'published files; all', manifest['measured_core_file_count'], 'measured core files retain their accepted hashes. Map payloads are excluded.')
