"""Replay a path certificate or rebuild and verify a finite-graph certificate."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import resource
import time

import numpy as np

from .semantic_constraint_core import ConstraintWorld, replay_edge
from .semantic_constraint_lattice import ResourceLattice
from .semantic_constraint_study import fresh, write_json, seal
from .semantic_map import sha256_file


def verify_artifacts(directory):
    hashes=json.loads((directory/'artifact_hashes.json').read_text())
    mismatches=[p for p,h in hashes.items() if sha256_file(directory/p)!=h]
    if mismatches:
        raise ValueError(f"artifact hash mismatch: {mismatches}")
    return len(hashes)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('path','graph'))
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--query',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    output=fresh(args.output)
    started=time.monotonic()
    artifacts=verify_artifacts(args.run)
    world=ConstraintWorld(args.inputs,args.query)
    if args.mode=='path':
        certificate=json.loads((args.run/'certificate.json').read_text())
        if certificate['input_npz_sha256']!=world.meta['npz_sha256']:
            raise ValueError('path input binding mismatch')
        edges=[replay_edge(c) for c in certificate['edges']]
        edge_hashes=all(e.certificate()['sample_hash']==c['sample_hash']
                        for e,c in zip(edges,certificate['edges']))
        audit,path=world.audit(edges)
        original=json.loads((args.run/'path.json').read_text())
        original=np.asarray([[p['x'],p['y'],p['yaw']] for p in original])
        path_exact=np.array_equal(path,original)
        valid=bool(audit['gate_passed'] and edge_hashes and path_exact and certificate['qualifying'])
        result={'mode':'path','verification_passed':valid,'audit':audit,
                'edge_sample_hashes_exact':edge_hashes,'saved_path_exact':path_exact}
    else:
        cfg=json.loads((args.run/'result.json').read_text())['graph_config']
        lattice=ResourceLattice(world,spacing=cfg['station_spacing_m'],lateral=cfg['lateral_spacing_m'],
                                extension=cfg['extension_m'],yaw_offsets=tuple(cfg['yaw_offsets']),
                                skip=cfg['max_station_skip'],lateral_step=cfg['lateral_neighbor_distance_m'])
        lattice.prepare_suffix_bounds()
        rebuilt_edges=np.asarray([(a,*e) for a,ns in lattice.adjacency.items() for e in ns],dtype=float)
        rebuilt_goals=np.asarray([(a,*e) for a,e in lattice.goal_connectors.items() if e is not None],dtype=float)
        with np.load(args.run/'graph_certificate.npz') as certificate:
            checks={'edges_exact':np.array_equal(rebuilt_edges,certificate['edges']),
                    'goal_edges_exact':np.array_equal(rebuilt_goals,certificate['goal_edges']),
                    'side_bound_exact':np.array_equal(lattice.suffix_side,certificate['side_upper']),
                    'target_bound_exact':np.array_equal(lattice.suffix_target,certificate['target_upper']),
                    'length_bound_exact':np.array_equal(lattice.suffix_length,certificate['length_lower'])}
        with np.load(args.run/'graph_nodes.npz') as nodes:
            checks['graph_vertices_exact']=np.array_equal(lattice.poses,nodes['poses'])
        n,c,t=world.semantic_counts(np.array([world.start]))
        side=5*c-4*n+lattice.suffix_side[0]
        target=2*t-n+lattice.suffix_target[0]
        checks['necessary_resource_gate_impossible']=bool(side<0 or target<=0)
        result={'mode':'graph','verification_passed':all(checks.values()),'checks':checks,
                'including_start_side_upper':float(side),'including_start_target_upper':float(target),
                'scope':'this_exact_finite_DAG_only; continuous_feasibility_not_proved'}
    result.update({'input_npz_sha256':world.meta['npz_sha256'],'verified_artifact_count':artifacts,
                   'wall_s':time.monotonic()-started,'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                   'source_hashes':{str(p):sha256_file(p) for p in (Path(__file__),Path(__file__).with_name('semantic_constraint_core.py'),Path(__file__).with_name('semantic_constraint_lattice.py'))}})
    write_json(output/'verification.json',result)
    seal(output)
    print(json.dumps(result,indent=2))
    return 0 if result['verification_passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
