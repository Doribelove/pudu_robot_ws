"""Independent strict replay of newly generated R2 candidate certificates."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from .semantic_constraint_core import ConstraintWorld, replay_edge
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_path_revisit_audit import audit_revisits
from .semantic_transition_contract import resample_path
from .semantic_transition_preflight import replay_controls, write_json
from .semantic_transition_r2_preflight import semantic_metrics, explicit_hard_audit


def render(world, points, metric, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    samples, station = resample_path(points)
    path = np.asarray([[p[k] for k in ('x','y','yaw')] for p in points])
    rows, cols, _ = world.cells(path)
    r0,r1 = max(0,int(rows.min())-20),min(world.map.height,int(rows.max())+21)
    c0,c1 = max(0,int(cols.min())-20),min(world.map.width,int(cols.max())+21)
    pixels = np.zeros_like(world.master,dtype=np.uint8)
    pixels[world.master>=253] = 1
    target = world.grids['correct'] & (world.grids['error']<=.5)
    pixels[target & (world.master<253)] = 2
    low = world.map.cell_to_world((r1-1,c0))
    high = world.map.cell_to_world((r0,c1-1))
    fig,axes = plt.subplots(1,2,figsize=(11,9),gridspec_kw={'width_ratios':[1,1.4]},layout='constrained')
    axes[0].imshow(pixels[r0:r1,c0:c1],extent=(low[0]-.025,high[0]+.025,low[1]-.025,high[1]+.025),
                   cmap=ListedColormap(['#f2f3f4','#36393c','#a5dbc1']),vmin=0,vmax=2,origin='upper')
    axes[0].plot(path[:,0],path[:,1],color='#176baa',linewidth=1.7,label='Generated path')
    axes[0].scatter([world.start[0],world.goal[0]],[world.start[1],world.goal[1]],c=['#ae2c38','#8854ad'],s=35,zorder=5)
    for pose,label in ((world.start,'Start'),(world.goal,'Goal')):
        axes[0].annotate(label,pose[:2],xytext=(8,0),textcoords='offset points',fontsize=9)
    axes[0].set_aspect('equal'); axes[0].set_title('Frozen map / green: target band'); axes[0].set_xlabel('x (m)');axes[0].set_ylabel('y (m)')
    sampled = np.asarray([[p[k] for k in ('x','y','yaw')] for p in samples])
    rr,cc,_ = world.cells(sampled)
    errors = world.grids['error'][rr,cc]
    axes[1].plot(station,errors,color='#176baa',linewidth=1,label='Lateral error')
    axes[1].axhline(.5,color='#ae2c38',linestyle='--',label='0.50 m threshold')
    left,right = metric['active_interval_m']
    axes[1].axvspan(0,left,color='#cccccc',alpha=.5)
    axes[1].axvspan(right,station[-1],color='#cccccc',alpha=.5)
    lane = metric['active_window']['classes']['lane']
    axes[1].set_title(f"Window: side={lane['correct_side_ratio']:.4f}, band={lane['target_band_ratio']:.4f}\nP50={lane['lateral_error_p50_m']:.3f} m; grey: transition")
    axes[1].set_xlabel('Full-path arc length (m)');axes[1].set_ylabel('Lateral error (m)');axes[1].legend(loc='upper right')
    fig.savefig(output/'path_and_semantics.png',dpi=170);plt.close(fig)


def run(inputs,query,candidate,output):
    output.mkdir(parents=False,exist_ok=False)
    world = ConstraintWorld(inputs,query)
    source = json.loads((candidate/'result.json').read_text())
    if source.get('input_npz_sha256') != world.meta['npz_sha256']:
        raise ValueError('candidate input binding mismatch')
    path_file, controls_file = candidate/'path.json',candidate/'controls.json'
    points = json.loads(path_file.read_text())
    controls = replay_controls(path_file,controls_file,'certificate')
    edges = [replay_edge(c) for c in json.loads(controls_file.read_text())['edges']]
    audit,path = world.audit(edges)
    metric = semantic_metrics(world,points)
    semantic_map = SemanticMapV1.load(inputs.parent/'conversion_v1/semantic_map_v1.json')
    if semantic_map.semantic_map_hash != world.meta['semantic_map_hash']:
        raise ValueError('semantic map mismatch')
    hard = explicit_hard_audit(world,points,semantic_map)
    revisit = audit_revisits(path)
    passed = bool(controls['control_replay_passed'] and audit['canonical']['final_valid_success']
                  and audit['padded_effective_master_collision_free'] and audit['exact_endpoint_xy_yaw']
                  and audit['same_lane_instance'] and audit['maximum_control_curvature_1pm']<=2.5
                  and controls['arc_length_m']<=world.bound_length and hard['hard_feature_gate_passed']
                  and metric['semantic_gate_passed'] and revisit['revisit_screen_passed'])
    result = {'query_id':query,'strict_replay_gate_passed':passed,'controls':controls,'safety':audit,
              'semantics':metric,'hard_features':hard,'revisit':revisit,
              'candidate':str(candidate),'candidate_result_sha256':sha256_file(candidate/'result.json'),
              'path_sha256':sha256_file(path_file),'controls_sha256':sha256_file(controls_file),
              'input_meta_sha256':sha256_file(inputs/f'{query}.json'),'input_npz_sha256':world.meta['npz_sha256'],
              'source_sha256':sha256_file(Path(__file__)),
              'created_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
              'online_run':False,'scope':'independent offline candidate replay; no online performance claim'}
    write_json(output/'verification.json',result)
    render(world,points,metric,output)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','candidate','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--query',required=True)
    args=parser.parse_args(argv)
    result=run(args.inputs,args.query,args.candidate,args.output)
    print(json.dumps({'query':args.query,'strict_replay_gate_passed':result['strict_replay_gate_passed']},indent=2))
    return 0 if result['strict_replay_gate_passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
