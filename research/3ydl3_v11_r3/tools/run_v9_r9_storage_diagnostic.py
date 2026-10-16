"""Original arena full10 exact progression with an outside-clock array ledger."""
import os
import subprocess
import sys
from pathlib import Path
from compare_v9_review import load, save, sha
from audit_v9_r9_source import verify
from run_free_full import registered_input

ROOT = Path(__file__).resolve().parents[1]
MAP_ID = 'arena_large_250x300_2d_005'


def main():
    output = ROOT / 'results/storage_diagnostic_source3'
    assert not output.exists(), 'Never overwrite or restart an existing diagnostic batch'
    protection = verify(3)
    save(ROOT / 'report/SOURCE3_STORAGE_PRE_NATIVE_PROTECTED_REVIEW.json', protection)
    directory, entry, queries = registered_input(MAP_ID)
    fixture = ROOT / 'datasets/moving_start_v1' / (MAP_ID + '.json')
    lock = ROOT / 'report/PLANNER_SOURCE_LOCK.json'
    assert load(lock)['source_cycle'] == 3
    protocol = {
        'version': '3YD-V9-r9', 'source_cycle': 3, 'diagnostic_only': True,
        'purpose': 'Owner ledger diagnosis of failed original arena exact all40 RSS gate',
        'map_id': MAP_ID, 'modes': ['AS', 'control', 'PLM'],
        'requests_each_mode': 40, 'expected_records': 120,
        'query_scope': 'all original ten; each initial,p20,p50,p80; no perturbation or resampling',
        'fixture': str(fixture), 'fixture_sha256': sha(fixture),
        'original_map_yaml': str(directory/'map.yaml'),
        'original_map_yaml_sha256': sha(directory/'map.yaml'),
        'original_fixed_queries_sha256': sha(directory/'queries.json'),
        'planning_lock_sha256': sha(lock), 'planning_files': 122,
        'storage_ledger_scope': 'Original --storage-ledger; prepared and after each request; no properties evaluated',
        'timing_not_used_for_formal_acceptance': True,
        'allocator_state_can_be_affected_by_diagnostic_inventory': True,
        'process_snapshot_not_request_peak': True,
        'limits': 'Array-owner ledger excludes Python containers, C++/ROS allocations and allocator-retained freed blocks',
        'CPU': 12, 'OMP_OPENBLAS_MKL_threads': 1,
        'L1_frozen': True, 'L2_search_native_connector_audits_unchanged': True,
        'original_AS_all40_memory_failure_preserved': True,
        'reference_failed_review_sha256': sha(ROOT/'report/SOURCE3_FULL_PROGRESS_REVIEW.json'),
        'output': str(output),
    }
    save(ROOT/'report/SOURCE3_STORAGE_DIAGNOSTIC_PROTOCOL.json', protocol)
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for mode in protocol['modes']:
        original = mode == 'AS'
        command = ['taskset', '-c', '12', sys.executable, str(ROOT/'tools/run_task_progress.py'),
                   '--map-id', MAP_ID, '--output', str(output/'moving'/mode/MAP_ID),
                   '--optimization-mode', 'AB', '--v9-mode', 'A', '--review-mode', 'S',
                   '--r2-mode', 'control', '--r3-mode', 'control',
                   '--r5-mode', 'control' if original else 'C',
                   '--r6-mode', 'control' if original else 'H',
                   '--r7-mode', 'control' if original else 'NT',
                   '--r8-mode', 'control' if original else 'OGR',
                   '--r9-mode', 'control' if original else mode,
                   '--workload', 'moving', '--repeats', '1', '--storage-ledger']
        print('DIAGNOSTIC BATCH', mode, MAP_ID, flush=True)
        subprocess.run(command, cwd=ROOT, env=env, check=True)


if __name__ == '__main__':
    main()
