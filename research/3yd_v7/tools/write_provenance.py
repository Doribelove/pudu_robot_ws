"""Bind saved r2 batches to their actual source snapshots and experiment flags.

The inherited benchmark writer labeled the initial batches V6-r0. This file
adds provenance without rewriting any raw protocol or run record.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAPS = ('pudu_wanda_3f', 'mengjia', '3yd_synth_multiloop_a_v1',
        '3yd_synth_twinwing_b_v1', '3yd_synth_trizone_c_v2')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    batches = [(ROOT / 'results/formal_r2' / map_id, True, False) for map_id in MAPS]
    batches += [(ROOT / 'results/formal_r2_combined' / map_id, True, True) for map_id in MAPS]
    batches += [(ROOT / 'results/repeat_checks/pudu_wanda_3f_Q08', True, True),
                (ROOT / 'results/repeat_checks/mengjia_Q07', True, True)]
    manifest = []
    for folder, resume, buckets in batches:
        protocol_path = folder / 'protocol.json'
        snapshot_path = folder / 'source_snapshot/hashes.json'
        raw_protocol = json.loads(protocol_path.read_text())
        hashes = json.loads(snapshot_path.read_text())
        def snap(name):
            return next(value for key, value in hashes.items() if key.endswith('/src/three_yd_v5/' + name))
        record = {
            'actual_experiment_version': '3YD-V6-r2',
            'inherited_raw_protocol_version': raw_protocol['version'],
            'lazy_first_phase_resume': resume,
            'dominance_count_buckets': buckets,
            'precompiled_l1_index': False,
            'source_snapshot_hashes_sha256': digest(snapshot_path),
            'topology_astar_snapshot_sha256': snap('topology_astar.py'),
            'recovery_snapshot_sha256': snap('recovery.py'),
            'runs_sha256': digest(folder / 'runs.jsonl'),
            'raw_protocol_sha256': digest(protocol_path),
            'source_snapshot_dir': str(folder / 'source_snapshot'),
            'note': 'Initial r2 benchmark writer inherited the V6-r0 label; raw files are unchanged.',
        }
        (folder / 'experiment_addendum.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
        manifest.append({'folder': str(folder), **record})
    output = ROOT / 'report/PROVENANCE.json'
    output.write_text(json.dumps({'schema': '3YD-V6-r2-provenance-v1', 'batches': manifest},
                                 ensure_ascii=False, indent=2) + '\n')
    print(f'{len(manifest)} batches bound to source snapshots')


if __name__ == '__main__':
    main()
