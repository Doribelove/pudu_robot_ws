#!/usr/bin/env python3
"""Import/resource contract only; does not instantiate/start a ROS planner."""
import hashlib
import json
from pathlib import Path
import sys
from trilayer_p6_runtime import RuntimeConfig,activate
root=Path(sys.argv[1]).resolve()
result=activate(RuntimeConfig('/test/global_costmap/trilayer_get_costmap',Path('/tmp/trilayer-p6-import-cache')),resource_root=root)
assert result['profile']=='p6' and result['arm']=='H-I'
assert result['IncrementalService'].__name__=='IncrementalService'
assert result['CurveSession'].__name__=='CurveSession'
paths=result['sources']
assert paths['three_yd_v13.schedule']['relative_path'].startswith('profiles/p6/src/')
assert paths['three_yd_v13.service']['relative_path'].startswith('base/src/')
assert paths['three_yd_v5.curve_session']['relative_path'].startswith('base/runtime/base/src/')
import three_yd_v5.p6_hint
assert str(Path(three_yd_v5.p6_hint.__file__).resolve()).startswith(str(root/'p6/profiles/p6/src'))
from three_yd_v5.curve_backend import CONFIG_PATH
assert CONFIG_PATH.is_file() and CONFIG_PATH.resolve().is_relative_to(root)
manifest=json.loads((root.parent/'SOURCE_PROVENANCE.json').read_text())
for rel,sha in manifest['sha256'].items():
    expected=manifest['patches'].get(rel,{}).get('patched_sha256',sha)
    assert hashlib.sha256((root/'p6'/rel).read_bytes()).hexdigest()==expected,rel
built=list((root/'p6').rglob('*.so'))
assert len(built)==19, [p.name for p in built]
assert (root/'p6/base/runtime/base/native/curve_nav2_stack').is_file()
# All loaded original-family modules must come from the selected private tree.
assert all((root/'p6'/p['relative_path']).is_file() for p in paths.values())
print('PASS real IncrementalService and CurveSession imported from private source')
print('PASS P6 overlay priority and',len(paths),'original-family module bindings')
print('PASS',len(manifest['sha256']),'provenance files; explicit service/native patches only')
print('PASS installed-layout config resolution; 19 source-built libraries plus native Nav2 worker')
