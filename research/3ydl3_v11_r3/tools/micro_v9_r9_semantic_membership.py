"""Actual original labels, function-only paired probes, no Native planning."""
import hashlib
import os
import time
from pathlib import Path
import cv2
import numpy as np
from run_task_progress import SemanticWorld, DefaultTopology
from three_yd_v5.v9_r9_membership import semantic_membership, membership_stats
from compare_v9_review import load, save

ROOT = Path(__file__).resolve().parents[1]


def main():
    output = ROOT / 'results/micro_source3_semantic_membership'
    output.mkdir(parents=True, exist_ok=False)
    os.environ['THREEYD_V9_MODE'] = 'A'
    os.environ['THREEYD_V9_R9_MODE'] = 'M'
    world = SemanticWorld(ROOT / 'inputs/corroborated_map.yaml', ROOT / 'inputs/semantic.json',
                          cache_root=output / 'world')
    topology = DefaultTopology(world, ROOT / 'inputs/topology.json', output / 'graph')
    labels = topology.subregions
    before = hashlib.sha256(labels.tobytes()).hexdigest()
    original = load(ROOT / 'results/formal_source2/fixed/control/mengjia/cases/Q07_1/l1_output.json')
    ids = {'original_Q07_route': original['region_sequence'],
           'original_lane_junction': [rid for rid, region in topology.regions.items()
                                      if region['kind'] in ('lane', 'junction_area')],
           'original_parking_regions': [rid for rid, region in topology.regions.items()
                                       if region['kind'] == 'parking_area']}
    result = {'source_cycle': 3, 'diagnostic_only': True, 'function_only': True,
              'formal_performance_claim': False, 'Native_planning': False,
              'map_hash': world.map.sha256, 'source_label_hash': before,
              'label_dtype': str(labels.dtype), 'shape': list(labels.shape),
              'label_min': int(labels.min()), 'label_max': int(labels.max()),
              'CPU': sorted(os.sched_getaffinity(0)), 'OpenCV_threads': cv2.getNumThreads(),
              'input_source': 'current original SemanticWorld and sealed DefaultTopology', 'groups': {}}
    for name, elements in ids.items():
        expected = np.isin(labels, elements)
        assert np.array_equal(semantic_membership(labels, elements), expected)
        values = {'original_ms': [], 'M_ms': []}
        for rep in range(7):
            order = ('original', 'M') if rep % 2 == 0 else ('M', 'original')
            for mode in order:
                started = time.perf_counter()
                current = np.isin(labels, elements) if mode == 'original' else semantic_membership(labels, elements)
                elapsed = (time.perf_counter() - started) * 1000
                assert np.array_equal(current, expected)
                values[mode + '_ms'].append(elapsed)
                del current
        result['groups'][name] = {'ids': elements, 'n_per_implementation': 7,
                                 'original_median_ms': float(np.median(values['original_ms'])),
                                 'M_median_ms': float(np.median(values['M_ms'])),
                                 'independent_output_exact': True, **values}
        del expected
    assert hashlib.sha256(labels.tobytes()).hexdigest() == before
    result['input_unchanged'] = True
    result['actual_membership'] = membership_stats()
    save(ROOT / 'report/MICRO_SOURCE3_MEMBERSHIP.json', result)
    print('actual labels', labels.shape, labels.dtype, result['label_min'], result['label_max'],
          'threads', result['OpenCV_threads'], flush=True)
    for key, group in result['groups'].items():
        print(key, group['original_median_ms'], group['M_median_ms'], flush=True)


if __name__ == '__main__':
    main()
