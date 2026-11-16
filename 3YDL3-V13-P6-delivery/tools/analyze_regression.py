"""Audit the fixed P6 release against original static inputs; never drop failures."""
import argparse
import csv
import hashlib
import importlib.util
import json
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
B = Path('/home/robot/workspaces/3ydl3_v12_r0_completion_b_20261020')
OLD = Path('/home/robot/文档/ChatGPT/dddd-任务/output/v12_taskbook_closeout_20261021')


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quant(values):
    a = np.asarray(values, float)
    return dict(zip(('P50', 'P95', 'P99', 'max'),
                    map(float, [*np.quantile(a, [.5, .95, .99]), max(a)]))) if len(a) else None


def statistics(rows):
    return {'n': len(rows), 'success': sum(r['success'] for r in rows),
            'exact': sum(r['exact'] for r in rows), 'tolerant': sum(r['tolerant'] for r in rows),
            'over_4s': sum(r['wall_s'] > 4 for r in rows),
            'failures': dict(Counter(r['failure_code'] for r in rows if not r['success'])),
            **{key: quant([r[key] for r in rows]) for key in
               ('wall_s', 'CPU_s', 'RSS_MiB', 'mean_busy_cores',
                'wall_s_per_reference_m', 'CPU_s_per_reference_m', 'RSS_MiB_per_reference_m')}}


def endpoint_error(a, b):
    return (float(np.linalg.norm(a[:2] - b[:2])),
            float(abs(np.arctan2(np.sin(a[2] - b[2]), np.cos(a[2] - b[2])))))


def endpoint_contract(record, outcome, points):
    """Recompute exact/tolerant identity using the frozen endpoint policy."""
    success = bool(record['success'])
    exact, tolerant = record.get('exact_endpoint_success', False), record.get('tolerant_endpoint_success', False)
    assert isinstance(exact, bool) and isinstance(tolerant, bool)
    assert int(exact) + int(tolerant) == int(success), 'ENDPOINT_CLASS_PARTITION'
    assert bool(outcome.get('exact_endpoint_success', False)) == exact
    assert bool(outcome.get('tolerant_endpoint_success', False)) == tolerant
    if not success:
        return
    detail = outcome['endpoint_tolerance']
    assert record['endpoint_tolerance'] == detail, 'ENDPOINT_METADATA_ROW_BINDING'
    assert detail['policy'] == 'exact_first_then_endpoint_tolerance_v1'
    assert detail['position_tolerance_m'] == .2 and detail['yaw_tolerance_deg'] == 30
    used = False
    for role, actual in (('start', points[0]), ('goal', points[-1])):
        requested = np.asarray(record[role], float)
        resolved = np.asarray(detail['resolved'][role], float)
        assert np.array_equal(np.asarray(detail['requested'][role], float), requested)
        assert requested.shape == resolved.shape == (3,) and np.isfinite(resolved).all()
        pe, ye = endpoint_error(requested, resolved)
        yaw_degrees = float(np.rad2deg(ye))
        used = used or pe > 1e-8 or yaw_degrees > 1e-6
        assert pe <= .2 + 1e-9 and yaw_degrees <= 30 + 1e-7
        epe, eye = endpoint_error(resolved, actual)
        assert epe <= 1e-8 and np.rad2deg(eye) <= 1e-6, 'RESOLVED_ENDPOINT_PATH_BINDING'
        assert abs(detail['offsets'][role]['position_m'] - pe) <= 1e-8
        assert abs(detail['offsets'][role]['yaw_deg'] - yaw_degrees) <= 1e-6
    assert detail['used'] == used == tolerant
    assert detail['success_class'] == ('tolerant_endpoints' if used else 'exact_endpoints')
    if used:
        first = detail['attempts'][0]
        assert not first['success'], 'TOLERANCE_WITHOUT_EXACT_FAILURE'
        assert list(first['start']) == list(record['start']) and list(first['goal']) == list(record['goal'])


def ack_contract(outcome):
    diagnostics = outcome['diagnostics']
    assert diagnostics['costmap_update_acknowledged'] is True
    assert diagnostics['costmap_ack_status'] in (
        'exact_effective_master_content_verified', 'reused_exact_effective_content_ack')
    assert diagnostics['costmap_ack_semantics'] == 'exact_effective_master'
    for key in ('costmap_ack_mismatch_cells', 'costmap_ack_hard_mismatch_cells',
                'costmap_ack_soft_mismatch_cells', 'costmap_ack_soft_exact_mismatch_cells',
                'costmap_ack_soft_exact_mismatch_ratio', 'costmap_ack_hash_mismatch',
                'costmap_ack_sequence_mismatch', 'costmap_ack_stale_roi_cells'):
        assert diagnostics[key] == 0, key
    assert int(diagnostics['costmap_ack_sequence']) > 0
    server_hash = diagnostics['server_costmap_content_hash']
    assert isinstance(server_hash, str) and len(server_hash) == 64
    assert all(c in '0123456789abcdef' for c in server_hash)
    if 'semantic_expected_server_content_hash' in diagnostics:
        assert server_hash == diagnostics['semantic_expected_server_content_hash']
    if diagnostics['costmap_ack_status'] == 'reused_exact_effective_content_ack':
        assert diagnostics['semantic_ack_reused'] and diagnostics['semantic_noop_complete_key_verified']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--formal', type=Path, required=True)
    args = ap.parse_args()
    formal = args.formal.resolve()
    maps = read(B / 'MAPS.json')['maps']
    bymap = {m['map_id']: m for m in maps}
    spec = importlib.util.spec_from_file_location('provenance_review', B / 'report/check_provenance.py')
    review = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(review)
    rows, proofs, problems, hashes = [], [], [], {}
    lock = read(ROOT / 'SOURCE_LOCK.json')
    assets = read(ROOT / 'docs/RUNTIME_ASSETS.json')
    formal_protocol = read(formal / 'PROTOCOL.json')
    runtime = Path(formal_protocol['package']).resolve()
    fixed_source = Path(lock['fixed_source']).resolve()

    def file_check(path, wanted, reason, **context):
        path = Path(path)
        try:
            actual = sha(path)
            hashes[str(path)] = actual
            assert actual == wanted, 'SHA256_MISMATCH'
            return True
        except Exception as exc:
            problems.append({'reason': reason, 'path': str(path), 'detail': repr(exc), **context})
            return False

    try:
        assert formal_protocol['version'] == lock['version'] == '3YDL3-V13-P6'
        assert formal_protocol['profile'] == lock['profile'] == 'p6'
        assert formal_protocol['arm'] == lock['l2_arm'] == 'H-I'
        assert formal_protocol['fixed_commit'] == lock['commit'] == assets['fixed_commit']
        assert formal_protocol['phase'] == 'formal' and formal_protocol['requested_rounds'] == 3
        assert Path(assets['runtime']).resolve() == runtime
    except Exception as exc:
        problems.append({'reason': 'formal_protocol_identity', 'detail': repr(exc)})
    file_check(ROOT / 'docs/RUNTIME_ASSETS.json', formal_protocol['runtime_assets_sha256'], 'runtime_assets_registration')
    file_check(B / 'MAPS.json', formal_protocol['map_registry_sha256'], 'original_map_registry')
    tools_lock = formal_protocol['tools_sha256']
    for rel, expected_sha in tools_lock.items():
        file_check(ROOT / rel, expected_sha, 'formal_tool_source')
    for m in maps:
        for path, expected_sha in m['input_sha256'].items():
            file_check(path, expected_sha, 'original_map_query_asset', map_id=m['map_id'])
    published = read(fixed_source / 'MANIFEST.json')['files']
    expected_runtime = {**published, **{p: item['sha256'] for p, item in assets['additions'].items()}}
    changed, runtime_changed = [], []
    for rel, expected_sha in lock['files'].items():
        if not file_check(fixed_source / rel, expected_sha, 'fixed_source_changed'):
            changed.append(rel)
        if not file_check(runtime / rel, expected_sha, 'runtime_published_source_changed'):
            runtime_changed.append(rel)
    for rel, item in assets['additions'].items():
        if not file_check(runtime / rel, item['sha256'], 'runtime_asset_changed'):
            runtime_changed.append(rel)
        file_check(item['source'], item['sha256'], 'runtime_asset_origin_changed')
    expected = {(rnd, m['map_id'], q) for rnd in range(1, 4) for m in maps for q in m['query_ids']}
    fixed_lengths = {}
    for m in maps:
        for q in m['query_ids']:
            p = OLD / 'formal_v1/round1' / m['map_id'] / 'B/cases' / (q + '_1') / 'l1_output.json'
            xy = np.asarray(read(p)['right_reference_polyline'])
            fixed_lengths[m['map_id'], q] = float(np.linalg.norm(np.diff(xy[:, :2], axis=0), axis=1).sum())
            hashes[str(p)] = sha(p)
    for runfile in sorted(formal.rglob('runs.jsonl')):
        batch = runfile.parent
        batch_records = list(map(json.loads, runfile.read_text().splitlines()))
        proto = read(batch / 'protocol.json')
        mid = proto['map_id']
        rnd = next(int(p[5:]) for p in batch.parts if p.startswith('round') and p[5:].isdigit())
        journal = read(batch.with_suffix('.supervisor.json'))
        if journal['returncode'] != 0 or not journal['cleanup']['complete']:
            problems.append({'batch': str(batch), 'reason': 'supervisor_or_cleanup'})
        if proto['vehicle'] != [.265, .225, .4] or proto['CPU'] != [14]:
            problems.append({'batch': str(batch), 'reason': 'vehicle_or_cpu'})
        try:
            assert proto['version'] == '3YDL3-V13-P6' and proto['workload'] == 'fixed'
            assert proto['l3_backend'] == 'independent' and proto['repeats'] == 1
            assert set(proto['actual_query_order']) == set(bymap[mid]['query_ids'])
            assert len(proto['actual_query_order']) == 10
            harness_sha = tools_lock['tools/p6_static_harness.py']
            assert proto['common_harness_sha256'] == harness_sha
            executed = read(batch / 'source_snapshot/executed_common_harness.json')
            assert executed['sha256'] == harness_sha
            file_check(batch / 'source_snapshot/executed_common_harness.py', harness_sha,
                       'batch_harness_source', batch=str(batch))
            snapshot = read(batch / 'source_snapshot/runtime_files.json')
            assert Path(snapshot['root']).resolve() == runtime and snapshot['sha256'] == expected_runtime
            for stage in ('BEFORE', 'AFTER'):
                evidence = read(batch / ('P6_RUNTIME_' + stage + '.json'))
                assert evidence['profile'] == 'p6' and evidence['arm'] == 'H-I'
                assert evidence['fixed_commit'] == lock['commit']
                assert not evidence['outside_algorithm_modules']
                assert Path(evidence['runtime']).resolve() == runtime
                modules = evidence['loaded_algorithm_modules']
                for name, item in modules.items():
                    path = Path(item['path']).resolve()
                    assert path.is_relative_to(runtime), (name, 'OUTSIDE_RUNTIME')
                    rel = str(path.relative_to(runtime))
                    assert item['sha256'] == expected_runtime[rel], (name, 'LOADED_SOURCE_HASH')
                if stage == 'AFTER':
                    for name, rel in {
                        'three_yd_v13.service': 'base/src/three_yd_v13/service.py',
                        'three_yd_v13.incremental': 'base/src/three_yd_v13/incremental.py',
                        'three_yd_v5.p6_hint': 'profiles/p6/src/three_yd_v5/p6_hint.py',
                    }.items():
                        assert Path(modules[name]['path']).resolve() == runtime / rel, name
        except Exception as exc:
            problems.append({'batch': str(batch), 'reason': 'batch_source_or_runtime_identity', 'detail': repr(exc)})
        # Original registered map/query identities are checked against historical inputs.
        bp = read(OLD / 'formal_v1/round1' / mid / 'B/protocol.json')
        for key in ('map_sha256', 'queries_sha256'):
            if proto[key] != bp[key]:
                problems.append({'batch': str(batch), 'reason': 'input_hash', 'key': key})
        try:
            proof = review.review(batch)
            assert proof['requests'] == len(batch_records), 'BATCH_PROVENANCE_REQUEST_COUNT'
            assert proof['success'] == sum(bool(r['success']) for r in batch_records), 'BATCH_PROVENANCE_SUCCESS_COUNT'
            proofs.append({k: v for k, v in proof.items() if k != 'records'})
        except Exception as exc:
            problems.append({'batch': str(batch), 'reason': 'curve_provenance', 'detail': repr(exc)})
        hashes[str(runfile)] = sha(runfile)
        for r in batch_records:
            case = batch / 'cases' / (r['query_id'] + '_1')
            outcome = read(case / 'outcome.json')
            hashes[str(case / 'outcome.json')] = sha(case / 'outcome.json')
            m = bymap[mid]
            reference = fixed_lengths[mid, r['query_id']]
            row = {'round': rnd, 'map_id': mid, 'map_name': m['name'], 'query_id': r['query_id'],
                   'area_m2': float(np.prod(m['dimensions_px']) * m['resolution_m'] ** 2),
                   'success': bool(r['success']), 'exact': bool(r.get('exact_endpoint_success', False)),
                   'tolerant': bool(r.get('tolerant_endpoint_success', False)),
                   'failure_code': r.get('failure_code', ''), 'wall_s': r['wall_ms'] / 1000,
                   'CPU_s': r['tree_cpu_ms'] / 1000, 'RSS_MiB': r['rss_peak_bytes'] / 2**20,
                   'length_m': r.get('length_m'), 'fixed_B_L1_m': reference, 'case': str(case)}
            row['mean_busy_cores'] = row['CPU_s'] / row['wall_s']
            for key in ('wall_s', 'CPU_s', 'RSS_MiB'):
                row[key + '_per_reference_m'] = row[key] / reference
            try:
                assert bool(outcome['success']) == row['success']
                assert not r.get('task_progress', {}).get('reused', False)
                assert r['version'] == '3YDL3-V13-P6' and r['map_id'] == mid
                ident = r['p6_runtime_identity']
                assert ident['service_class'] == 'three_yd_v13.service.IncrementalService'
                assert ident['arm'] == 'H-I' and ident['dynamic_version'] == 0
                assert ident['event_repair_requested'] is False
                if row['success']:
                    assert ident['controller_class'] == 'three_yd_v13.incremental.IncrementalController'
                else:
                    assert ident['controller_class'] in (None, 'three_yd_v13.incremental.IncrementalController')
                original = next(q for q in m['poses'] if q['id'] == r['query_id'])
                def pose(v):
                    return [v[k] for k in ('x', 'y', 'yaw_rad')] if isinstance(v, dict) else v
                assert list(r['start']) == pose(original['start'])
                assert list(r['goal']) == pose(original['goal'])
                if row['success']:
                    assert outcome['independent_audit']['valid']
                    assert outcome['constraint_audit']['steering_jump_valid']
                    assert outcome['constraint_audit'].get('traffic', {}).get('valid', True)
                    ack_contract(outcome)
                    assert outcome['curve_l3']['certificate']['max_curvature_bound_1pm'] <= 2.5
                    pts = np.load(case / 'path.npy')
                    assert pts.ndim == 2 and pts.shape[1] == 3 and np.isfinite(pts).all()
                    assert abs(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1).sum() - row['length_m']) < 1e-6
                    for role, pt in (('start', pts[0]), ('goal', pts[-1])):
                        pe, ye = endpoint_error(pt, np.asarray(r[role]))
                        assert pe <= .2 + 1e-7 and ye <= np.deg2rad(30) + 1e-7
                    endpoint_contract(r, outcome, pts)
                    hashes[str(case / 'path.npy')] = sha(case / 'path.npy')
                else:
                    endpoint_contract(r, outcome, None)
                    assert row['failure_code']
                    assert not (case / 'path.npy').exists()
            except Exception as exc:
                problems.append({'case': str(case), 'reason': 'output_contract', 'detail': repr(exc)})
            rows.append(row)
    keys = [(r['round'], r['map_id'], r['query_id']) for r in rows]
    if len(keys) != len(set(keys)) or set(keys) != expected:
        problems.append({'reason': 'coverage', 'missing': sorted(expected - set(keys)), 'duplicates': len(keys) - len(set(keys))})
    if sum(p['requests'] for p in proofs) != len(rows) or sum(p['success'] for p in proofs) != sum(r['success'] for r in rows):
        problems.append({'reason': 'provenance_record_count', 'proof_requests': sum(p['requests'] for p in proofs),
                         'row_requests': len(rows), 'proof_successes': sum(p['success'] for p in proofs)})
    result = {'version': '3YDL3-V13-P6', 'profile': 'p6', 'L2_arm': 'H-I',
              'formal_root': str(formal), 'all_requests': statistics(rows),
              'successful_requests': statistics([r for r in rows if r['success']]),
              'by_map': {m['map_id']: statistics([r for r in rows if r['map_id'] == m['map_id']]) for m in maps},
              'normalization': 'fixed completion-B first-round original-query L1 reference length; absolute totals retained',
              'records': rows}
    report = ROOT / 'report'
    (report / 'REGRESSION_RESULTS.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    with (report / 'p6_static_requests.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    audit = {'formal_requests': len(rows), 'expected_requests': 300,
             'policy_successes': sum(r['success'] for r in rows),
             'audited_successes': sum(p['success'] for p in proofs), 'source_unchanged': not changed,
             'runtime_published_and_assets_unchanged': not runtime_changed,
             'contract_problems': problems, 'contract_audit_passed': not problems,
             'functional_gate_passed': not problems and len(rows) == 300 and all(r['success'] for r in rows),
             'model_proofs': proofs}
    (report / 'REGRESSION_AUDIT.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2) + '\n')
    (report / 'RAW_DATA_MANIFEST.json').write_text(json.dumps(hashes, indent=2) + '\n')
    print(json.dumps({k:v for k,v in audit.items() if k != 'model_proofs'}, ensure_ascii=False, indent=2))
    print(json.dumps(result['all_requests'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
