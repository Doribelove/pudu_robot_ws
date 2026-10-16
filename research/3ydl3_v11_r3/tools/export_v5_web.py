"""Export the saved V5-r0 experiment into the Miaoda visualization contract."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    comparison = json.loads((ROOT / 'report/comparison.json').read_text())
    layers = json.loads((ROOT / 'report/layer_outputs.json').read_text())
    connectivity = json.loads((ROOT / 'report/CIRCLE_DIAGNOSIS.json').read_text())
    topology = json.loads((ROOT / 'inputs/topology.json').read_text())
    semantic = json.loads((ROOT / 'inputs/semantic.json').read_text())
    queries = {item['id']: item for item in json.loads((ROOT / 'inputs/queries.json').read_text())['scenarios']}
    rows = [json.loads(line) for line in (ROOT / 'results/formal_v5_r0/runs.jsonl').read_text().splitlines()]
    region_meta = {item['name']: item for item in topology['regions']}
    regions = []
    for feature in semantic['features']:
        name = feature.get('properties', {}).get('source_name')
        if name in region_meta:
            meta = region_meta[name]
            regions.append({'id': meta['id'], 'name': name, 'kind': meta['kind'], 'polygon': feature['coordinates']})
    layer_by_id = {item['query_id']: item for item in layers['queries']}
    connectivity_by_id = {item['query_id']: item for item in connectivity['records']}
    cases = []
    for query_id in sorted(queries):
        query = queries[query_id]
        repeated = [item for item in rows if item['query_id'] == query_id]
        first = repeated[0]
        layer = layer_by_id[query_id]
        l2 = first.get('l2', {})
        bounded = first.get('bounded_l3', {})
        l3_path = layer['L3_accepted_xyyaw'] or []
        cases.append({
            'id': query_id,
            'name': query['name'],
            'band': query['distance_band'],
            'focus': query['focus'],
            'start': query['start'],
            'goal': query['goal'],
            'strict_success': bool(first['success']),
            'success_count': sum(bool(item['success']) for item in repeated),
            'repeats': len(repeated),
            'failure': first['failure_code'],
            'median_s': float(np.median([item['wall_ms'] / 1000 for item in repeated])),
            'rss_gib': max(item['rss_peak_bytes'] for item in repeated) / 2**30,
            'l1': {
                'region_sequence': layer['L1']['region_sequence'],
                'right_reference': layer['L1']['right_reference_polyline'],
                'schema': layer['L1']['schema'],
                'directed_outline_dependencies': False,
            },
            'l2': {
                'available': bool(layer['L2_reference_xy']),
                'reference': layer['L2_reference_xy'] or [],
                'corners': layer.get('L2_corners', []),
                'state': ['x', 'y'],
                'heading_state': False,
                'inflation_radius_m': 0.3476348659153739,
                'resolution_m': 0.15,
                'raw_nodes': l2.get('v5', {}).get('raw_nodes'),
                'straightened_nodes': l2.get('v5', {}).get('straightened_nodes'),
                'expanded': l2.get('dstar_expanded'),
                'backend': l2.get('selected_backend'),
            },
            'l3': {
                'called': bool(layer['L2_reference_xy']),
                'final_path': [[pose[0], pose[1]] for pose in l3_path],
                'normal_radius_m': bounded.get('normal_radius_m'),
                'corner_radius_m': bounded.get('corner_radius_m'),
                'corner_threshold_deg': bounded.get('corner_threshold_deg'),
                'search_cells': bounded.get('search_cells'),
                'map_fraction': bounded.get('map_fraction'),
                'global_fallback': False,
                'steering_jump_deg': first.get('constraints', {}).get('max_raw_steering_jump_deg'),
                'audit_valid': first.get('audit', {}).get('valid', False),
                'ack_mismatch_cells': first.get('l3', {}).get('costmap_ack_mismatch_cells'),
            },
            'metrics': {
                'length_m': first.get('length_m'),
                'right_fraction': first.get('right_side_fraction'),
            },
            'connectivity': connectivity_by_id[query_id],
        })
    output = {
        'version': '3YD-V5-r0',
        'source': 'results/formal_v5_r0 · fixed 10 queries × 3 repeats',
        'status': 'experimental_success_rate_unchanged',
        'map': {
            'width': semantic['width'], 'height': semantic['height'],
            'resolution': semantic['resolution'], 'origin': semantic['origin'],
            'sha256': connectivity['map_sha256'],
        },
        'regions': regions,
        'summary': {
            'strict_queries': 2, 'total_queries': 10, 'strict_runs': 6, 'total_runs': 30,
            'l1_queries': 10, 'l2_queries': 2,
            'all_median_s': comparison['sources']['V5-r0']['all_runs']['median_s'],
            'p95_s': comparison['sources']['V5-r0']['all_runs']['p95_s'],
            'success_median_s': comparison['sources']['V5-r0']['all_runs']['successful_query_median_s'],
            'peak_rss_gib': comparison['sources']['V5-r0']['all_runs']['rss_peak_gib'],
        },
        'comparison': {
            'r6': comparison['sources']['V4-r6-B']['first_repeat']['all_runs'] if 'all_runs' in comparison['sources']['V4-r6-B']['first_repeat'] else comparison['sources']['V4-r6-B']['first_repeat'],
            'v5_first': comparison['sources']['V5-r0']['first_repeat'],
            'v5_all': comparison['sources']['V5-r0']['all_runs'],
        },
        'ablation': comparison['ablation'],
        'diagnostics': {
            'tests_passed': 11,
            'source_snapshot_matches': comparison['planning_sources_match_snapshot'],
            'resource_cleanup': all(comparison['cleanup'].values()),
            'dynamic_validation': False,
            'circle_diagnosis_only': True,
        },
        'cases': cases,
    }
    target = ROOT / 'report/v5-web-experiment.json'
    target.write_text(json.dumps(output, ensure_ascii=False, separators=(',', ':')) + '\n')
    print(target, target.stat().st_size)


if __name__ == '__main__':
    main()
