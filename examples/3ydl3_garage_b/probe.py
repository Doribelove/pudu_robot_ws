# SPDX-License-Identifier: Apache-2.0
"""Validate the isolated import closure without executing a navigation request."""
import importlib
import json
from pathlib import Path
import sys

from demo import R4, FIXED, MAP_ID
sys.path.insert(0, R4 + '/tools')
for name in ['numpy', 'scipy', 'cv2', 'clarabel', 'osqp', 'rclpy', 'run_crossmap_v10']:
    module = importlib.import_module(name)
    print(name, getattr(module, '__version__', 'ROS'), module.__file__)
sys.path.insert(0, FIXED)
from fixed10_binding import resolve
inputs, binding, queries = resolve(MAP_ID)
assert len(queries['scenarios']) == 10
assert not Path('/home/robot/.ssh').exists()
assert not Path('/home/robot/workspaces/3ydl3_v11_r5_routequality_b_20261019').exists()
print(json.dumps({'map': binding['map_id'], 'queries': len(queries['scenarios']),
                  'host_home_hidden': True, 'inputs': str(inputs)}))
