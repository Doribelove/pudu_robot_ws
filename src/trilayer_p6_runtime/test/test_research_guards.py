#!/usr/bin/env python3
"""Real private modules, no ROS launch; unsupported constructors fail first."""
from pathlib import Path
import sys
from trilayer_p6_runtime import activate, RuntimeConfig
from trilayer_p6_runtime.guards import BLOCKED, UnsupportedResearchFeature
result=activate(RuntimeConfig('/test/trilayer_get_costmap',Path('/tmp/trilayer-guards-cache')),
                resource_root=Path(sys.argv[1]))
import importlib
adaptive=importlib.import_module('arena_evaluation.two_layer_v1_formal_benchmark')
production=importlib.import_module('arena_3d_v1.production_l1')
assert production.adaptive.build_adaptive_corridor_mask is adaptive.build_adaptive_corridor_mask
assert not getattr(adaptive.build_adaptive_corridor_mask,'_trilayer_unsupported',False)
crop=importlib.import_module('three_yd_v5.r11_crop')
# Passing no arguments would fail with TypeError in the old constructor. The
# explicit guard must instead reject before allocating template/session objects.
try:crop.StorageCropSession()
except UnsupportedResearchFeature:pass
else:raise AssertionError('crop construction was not blocked')
assert 'native_snapshot' not in sys.modules
count=0
for name,functions in BLOCKED.items():
    module=importlib.import_module(name)
    for function in functions:
        if not hasattr(module,function):continue
        try:getattr(module,function)()
        except UnsupportedResearchFeature:count+=1
        else:raise AssertionError(name+'.'+function+' not blocked')
assert result['guards']['adaptive_corridor_function_unchanged']
print('PASS storage crop rejected before construction/native_snapshot/ROS launch')
print('PASS',count,'benchmark entrypoints rejected; production corridor function identity preserved')
print('PASS actual P6 service and session class imports still succeed')
