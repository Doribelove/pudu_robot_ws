# SPDX-License-Identifier: Apache-2.0
"""Maintainer-only manifest update; users run demo.py --verify-only."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
         for p in sorted(root.rglob('*')) if p.is_file()
         and p.relative_to(root).parts[0] not in ['.venv', '.native-build', 'log', 'output']
         and not any(s in p.relative_to(root).parts for s in ['__pycache__', '.pytest_cache'])
         and p.name != 'MANIFEST.json'}
manifest = {'version': '3YDL3-V11-r4-public-demo.1',
            'map_pgm_sha256': json.loads((root / 'SOURCE_PROVENANCE.json').read_text())['map_sha256'],
            'files': files}
(root / 'MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
print('Manifest:', len(files), 'files')
