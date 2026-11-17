#!/usr/bin/env python3
"""Read-only independent-release layout and solver source hash gate."""
from pathlib import Path
import hashlib,json,sys,xml.etree.ElementTree as ET
root=Path(sys.argv[1]).resolve();src=root/'src'
expected={'trilayer_planner_service','trilayer_nav2_planner','trilayer_p6_runtime',
 'trilayer_global_planner','trilayer_delta_codec','trilayer_l1_core','trilayer_clarabel_vendor',
 'trilayer_native_helpers','trilayer_costmap_bridge','trilayer_l3_core',
 'trilayer_master_snapshot','trilayer_osqp_vendor'}
found={}
for f in src.glob('*/package.xml'):
 name=ET.parse(f).findtext('name');assert name not in found,('duplicate',name)
 found[name]=str(f.relative_to(root))
assert set(found)==expected,{'missing':sorted(expected-set(found)),'unexpected':sorted(set(found)-expected)}
# Runtime binaries must be built here, not checked into this source layout.
for suffix in ('.so','.whl','.a','.o'):
 assert not list(src.rglob('*'+suffix)),('prebuilt artifact in src',suffix)
osqp=src/'trilayer_osqp_vendor';manifest=json.loads((osqp/'SOURCES_AND_LICENSES.json').read_text())
for name,record in manifest['sources'].items():
 archive=osqp/'sources'/record['filename'];assert hashlib.sha256(archive.read_bytes()).hexdigest()==record['sha256'],name
clarabel=src/'trilayer_clarabel_vendor';record=json.loads((clarabel/'SOURCE_MANIFEST.json').read_text())
assert hashlib.sha256((clarabel/'sources'/record['source_archive']).read_bytes()).hexdigest()==record['sha256']
evidence=root/'.ci_ws/evidence';evidence.mkdir(parents=True,exist_ok=True)
(evidence/'source-layout.json').write_text(json.dumps({'packages':found,'solver_source_hashes_verified':True,'prebuilt_runtime_artifacts':False},indent=2)+'\n')
print('PASS independent 12-package layout and solver source archive hashes')
