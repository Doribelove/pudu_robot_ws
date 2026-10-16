"""Check current R8 source, original inputs/audits/native, and archived cycles."""
import argparse
from pathlib import Path
from compare_v9_review import load, save
from compare_v9_r7_all import sha, FROZEN
from compare_v8_r3 import audit_ast
from audit_v9_r6_connector import audit_connector

ROOT = Path(__file__).resolve().parents[1]
PARENT = Path('/home/robot/workspaces/3yd_v9_r7_storage_batch_20261009')
PARENT_RELEASE = '4fea7c7ac4b94bd183b6b12ad79817170acb00cbed6defed8d66286bad3c9549'


def main():
    p = argparse.ArgumentParser(); p.add_argument('--source-cycle', type=int, required=True)
    a = p.parse_args(); out = {'source_cycle': a.source_cycle, 'checks': {}}
    checks = out['checks']
    assert sha(PARENT/'RELEASE.json') == PARENT_RELEASE
    files = load(PARENT/'RELEASE.json')['files']
    for f, expected in files.items(): assert sha(PARENT/f) == expected, f
    checks['R7_release_files'] = len(files)
    for name in ('PLANNER_SOURCE_LOCK.json', 'PRODUCTION_ENTRY_LOCK.json', 'EXTERNAL_AUDIT_LOCK.json'):
        data = load(ROOT/'report'/name); files = data.get('planning_source_files', data)
        if name == 'PLANNER_SOURCE_LOCK.json': assert data['source_cycle'] == a.source_cycle
        for f, expected in files.items(): assert sha(Path(f) if Path(f).is_absolute() else ROOT/f) == expected, f
        checks[name] = len(files)
    lock = load(ROOT/'L1_LOCK.json')
    for section in ('source_files', 'input_files', 'pixel_input_files'):
        for f, expected in lock[section].items(): assert sha(ROOT/f) == expected, f
        checks[section] = len(lock[section])
    for f in ('src/three_yd_v4/controller.py', 'src/three_yd_v4/r5_controller.py', 'src/three_yd_v5/controller.py'):
        assert audit_ast(ROOT/f) == audit_ast(FROZEN/f), f
    checks['audit_AST_same'] = True
    for f in ('src/three_yd_v5/task_progress.py', 'native/libcold_dstar.so', 'native/cold_dstar.cpp'):
        assert sha(ROOT/f) == sha(FROZEN/f), f
    checks['TaskProgress_and_L2_same'] = True
    checks['connector'] = audit_connector(ROOT, FROZEN)
    natives = [f for f in (PARENT/'native').rglob('*') if f.is_file()]
    natives += [f for f in (PARENT/'install').rglob('*.so') if f.is_file()]
    for f in natives: assert sha(ROOT/f.relative_to(PARENT)) == sha(f), f
    checks['Native_files'] = len(natives)
    parent = load(ROOT/'report/PARENT_PLANNER_SOURCE_LOCK.json')['planning_source_files']
    # The only inherited changes are explicit L3/data-storage hooks, version
    # environment and input/mode reporting. L1/L2 modules remain original.
    allowed = {'env.bash', 'src/three_yd_v5/controller.py', 'src/three_yd_v5/fast_costmap.py',
               'src/three_yd_v5/fullroute.py', 'src/three_yd_v5/r4_tiles.py', 'src/three_yd_v5/v9_maps.py',
               'tools/run_progress_events.py', 'tools/run_task_progress.py'}
    if a.source_cycle == 5: allowed.add('tools/run_v9_selected.py')
    changed = {f for f, expected in parent.items() if sha(ROOT/f) != expected}
    assert changed == allowed, changed
    checks['changed_inherited_files'] = sorted(changed)
    for cycle in range(a.source_cycle + 1):
        archive = ROOT/'report/source_cycles'/f'source{cycle}'
        data = load(archive/'PLANNER_SOURCE_LOCK.json')
        assert data['source_cycle'] == cycle
        for f, expected in data['planning_source_files'].items():
            assert sha(archive/'planning_source'/f) == expected, (cycle, f)
        checks[f'archived_source{cycle}'] = len(data['planning_source_files'])
    out['tests'] = (ROOT/'report'/f'tests_source{a.source_cycle}_all.txt').read_text().splitlines()[-1]
    assert 'passed' in out['tests'] and 'failed' not in out['tests']
    save(ROOT/'report'/f'PROTECTED_SOURCE{a.source_cycle}_REVIEW.json', out)
    print('PROTECTED', a.source_cycle, 'PASS', checks, out['tests'])


if __name__ == '__main__': main()
