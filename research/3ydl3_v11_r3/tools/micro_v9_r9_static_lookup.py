"""Lookup-only evidence; exhaustive bytes, original map shapes, no end-to-end claim."""
import os
import time
from pathlib import Path
import numpy as np
import cv2
from arena_evaluation.semantic_costmap_composer import occupancy_to_static_layer
from compare_v9_review import save

ROOT = Path(__file__).resolve().parents[1]


def main():
    original_threads = cv2.getNumThreads()
    cv2.setNumThreads(1)
    table = occupancy_to_static_layer(np.arange(256, dtype=np.uint8).view(np.int8))
    assert table.dtype == np.dtype(np.uint8) and table.shape == (256,)
    exhaustive = np.arange(256, dtype=np.uint8).reshape(16, 16)
    assert np.array_equal(table[exhaustive], cv2.LUT(exhaustive, table))
    # Different strides/shapes check algebraic behavior; a candidate would
    # only use its explicit supported ndarray guards and original fallback.
    cases = [exhaustive, exhaustive.T, exhaustive[::-1], exhaustive[:, ::2],
             np.array([[0]], np.uint8), np.array([[255]], np.uint8)]
    assert all(np.array_equal(table[a], cv2.LUT(a, table)) for a in cases)
    result = {'source_cycle': 1, 'diagnostic_only': True, 'function_only': True,
              'planner_code_changed': False, 'all256_original_conversion_values_identical': True,
              'exhaustive_stride_cases': len(cases), 'OpenCV_original_threads': original_threads,
              'OpenCV_benchmark_threads': 1, 'CPU': sorted(os.sched_getaffinity(0)),
              'input': 'all256-byte tiled probe at original map dimensions; NOT actual composed support distribution',
              'not_formal_planning_performance': True, 'maps': {}}
    for mid, shape in [('mengjia', (2251, 7124)), ('arena_large_250x300_2d_005', (6000, 5000))]:
        a = np.resize(np.arange(256, dtype=np.uint8), shape)
        before = table[a]
        after = cv2.LUT(a, table)
        assert np.array_equal(before, after)
        times = {'original_numpy_table': [], 'opencv_LUT': []}
        for rep in range(6):
            for mode in (list(times) if rep % 2 == 0 else list(reversed(times))):
                started = time.perf_counter()
                out = table[a] if mode == 'original_numpy_table' else cv2.LUT(a, table)
                times[mode].append((time.perf_counter() - started) * 1000)
                assert np.array_equal(out, before)
        result['maps'][mid] = {'shape': shape, 'input_bytes': a.nbytes,
                               'n_per_implementation': 6, 'all_outputs_exact': True,
                               'function_wall_ms': times,
                               'median_change_pct': 100 * (np.median(times['opencv_LUT']) / np.median(times['original_numpy_table']) - 1)}
    save(ROOT / 'report/STATIC_LOOKUP_MICRODIAGNOSTIC_SOURCE1.json', result)
    print('Exact byte lookup across all256 values/strides/shapes; NOT planner acceptance')
    for mid, data in result['maps'].items():
        print(mid, data['function_wall_ms'], data['median_change_pct'])


if __name__ == '__main__':
    main()
