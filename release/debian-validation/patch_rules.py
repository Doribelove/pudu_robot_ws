#!/usr/bin/python3
"""Explicit local validation delta; refuse unexpected Bloom rule shapes."""
from pathlib import Path
import argparse,difflib,re
p=argparse.ArgumentParser();p.add_argument('package_dir',type=Path);p.add_argument('name');p.add_argument('--solver-root');a=p.parse_args();f=a.package_dir/'debian/rules';before=f.read_text();s=before
assert s.count('dh_auto_test || true')==1,'Bloom template changed: review manually, do not assume tests are strict'
s=s.replace('dh_auto_test || true','CTEST_PARALLEL_LEVEL=1 dh_auto_test --max-parallel=1')
s=s.replace('echo -- Running tests. Even if one of them fails the build is not canceled.','echo -- Local validation: tests are mandatory and failures stop the build.')
assert s.count('$(BUILD_TESTING_ARG)')==1
args=['-DBUILD_TESTING=ON','-DPython3_EXECUTABLE=/usr/bin/python3']
flags={'trilayer_l3_core':'TRILAYER_L3_SOLVER_DEPS','trilayer_p6_runtime':'TRILAYER_P6_TEST_SOLVER_ROOT','trilayer_global_planner':'TRILAYER_ENTRY_TEST_SOLVER_ROOT','trilayer_planner_service':'TRILAYER_SERVICE_TEST_SOLVER_ROOT','trilayer_nav2_planner':'TRILAYER_NAV2_TEST_SOLVER_ROOT'}
if a.name in flags:
 assert a.solver_root and re.fullmatch(r'/[A-Za-z0-9_./+-]+',a.solver_root)
 args.append('-D'+flags[a.name]+'='+a.solver_root)
s=s.replace('$(BUILD_TESTING_ARG)',' '.join(args)+' $(BUILD_TESTING_ARG)')
(a.package_dir/'debian/rules.bloom-original').write_text(before)
(a.package_dir/'debian/LOCAL_VALIDATION_RULES.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),s.splitlines(True),fromfile='Bloom-generated rules',tofile='local strict validation rules')))
f.write_text(s)
