"""Regression and adversarial tests for the offline constraint certificate."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation.semantic_constraint_core import (
    CropMap, dense_interpolate, dubins_choices, dubins_edge,
    dubins_edge_from_parameters, replay_edge, ConstraintWorld,
)
from arena_evaluation.semantic_constraint_lattice import Labels
from arena_evaluation.semantic_constraint_study import fresh, write_json
from arena_evaluation.semantic_path_audit import SemanticPathAuditor
from arena_evaluation.semantic_rasterizer import grid_hash


def test_precomputed_dubins_choice_preserves_exact_edge():
    start = (0.0, 0.0, 0.25)
    goal = (1.3, 0.7, 1.1)
    for index, (word, params) in enumerate(dubins_choices(start, goal, 0.401)):
        original = dubins_edge(start, goal, 0.401, index)
        precomputed = dubins_edge_from_parameters(start, goal, 0.401, word, params)
        assert np.array_equal(original.samples, precomputed.samples)
        assert original.certificate() == precomputed.certificate()


def test_fast_dense_edge_certificate_is_conservative_about_neighborhood():
    world = object.__new__(ConstraintWorld)
    shape = (9, 9)
    world.map = SimpleNamespace(
        height=9, width=9, distance_m=np.full(shape, 2.0, dtype=np.float32),
    )
    world.selected = [4]
    world.master = np.zeros(shape, dtype=np.uint8)
    world.grids = {
        "labels": np.full(shape, 4, dtype=np.int16),
        "allowed": np.ones(shape, dtype=bool),
        "hard": np.zeros(shape, dtype=bool),
    }
    world.safe_threshold = 0.42
    world.cells = lambda samples: (
        np.full(len(samples), 4, dtype=np.int64),
        np.full(len(samples), 4, dtype=np.int64),
        np.ones(len(samples), dtype=bool),
    )
    samples = np.asarray([[0.0, 0.0, 0.0], [0.01, 0.0, 0.1]])
    assert world.fast_dense_edge_certificate(samples)
    world.grids["allowed"][3, 3] = False
    assert not world.fast_dense_edge_certificate(samples)
    world.grids["allowed"][3, 3] = True
    world.map.distance_m[4, 4] = 0.43
    assert not world.fast_dense_edge_certificate(samples)


def test_constraint_world_accepts_only_hash_bound_in_memory_master(tmp_path):
    shape = (9, 9)
    master = np.zeros(shape, dtype=np.uint8)
    labels = np.full(shape, 4, dtype=np.int16)
    arrays = {
        "master": master, "occupancy": np.zeros(shape, dtype=np.int8),
        "allowed": np.ones(shape, dtype=bool), "labels": labels,
        "error": np.zeros(shape, dtype=np.float32),
        "correct": np.ones(shape, dtype=bool),
        "right": np.zeros(shape, dtype=np.float32),
        "left": np.ones(shape, dtype=np.float32),
        "hard": np.zeros(shape, dtype=bool),
        "no_stopping": np.zeros(shape, dtype=bool),
    }
    meta = {
        "query": {"query_id": "q", "start": [0.1, 0.1, 0.0],
                  "goal": [0.3, 0.1, 0.0], "category": "lane", "seed": 1},
        "map": {"height": 9, "width": 9, "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0], "image_path": str(tmp_path / "map.pgm")},
        "selected_lane_labels": [4], "expected_master_hash": grid_hash(master),
    }
    world = ConstraintWorld(
        tmp_path, "q", meta_override=meta, arrays_override=arrays,
        master_override=master,
    )
    assert world.input_path is None
    assert np.array_equal(world.master, master)
    changed = master.copy()
    changed[0, 0] = 1
    with pytest.raises(ValueError, match="hash mismatch"):
        ConstraintWorld(
            tmp_path, "q", meta_override=meta, arrays_override=arrays,
            master_override=changed,
        )
def test_resource_dominance_retains_incomparable_histories_at_same_pose():
    labels=Labels()
    a=labels.add(9,-1,3.,-2,10)
    b=labels.add(9,-1,4.,8,3)
    assert a is not None and b is not None
    assert labels.frontiers[9]==[a,b]
    assert labels.add(9,-1,4.5,7,2) is None
    c=labels.add(9,-1,2.,10,12)
    assert labels.frontiers[9]==[c]
    assert not labels.active[a] and not labels.active[b]


def test_dominance_proof_holds_for_all_bounded_suffix_resources():
    # For any suffix, a dominating prefix preserves both exact inequalities.
    for suffix_n in range(25):
        for suffix_c in range(suffix_n+1):
            for suffix_t in range(suffix_c+1):
                ds=5*suffix_c-4*suffix_n
                dt=2*suffix_t-suffix_n
                if -4+ds>=0 and -3+dt>0:
                    assert -2+ds>=0 and -1+dt>0


def test_progress_is_part_of_vertex_identity_no_cross_station_dominance():
    labels=Labels()
    a=labels.add(1,-1,1.,100,100)
    b=labels.add(2,-1,2.,-20,-20)
    assert labels.active[a] and labels.active[b]


def test_exact_off_bin_goal_yaw_and_replay_without_duplicate_segments():
    start=(-26.350998999999995,21.066896,-math.pi/2)
    goal=(-25.800998999999997,-33.633104,-1.4876550949064484)
    edge=dubins_edge(start,goal)
    assert tuple(edge.samples[-1])==goal
    assert np.array_equal(edge.samples,replay_edge(edge.certificate()).samples)
    path=np.vstack((start,edge.samples))
    length=np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1)
    assert length.min()>1e-10
    assert length.max()<=.025
    tampered=edge.certificate()
    tampered["params"]=tuple(x+.001 for x in tampered["params"])
    with pytest.raises(ValueError,match="cannot be replayed"):
        replay_edge(tampered)


def test_edge_counts_are_identical_to_unchanged_semantic_sampler():
    a=(0.,0.,0.)
    b=(2.1,1.,.3)
    c=(4.,1.5,.12)
    first,second=dubins_edge(a,b),dubins_edge(b,c)
    path=np.vstack((a,first.samples,second.samples))
    sampled=SemanticPathAuditor._samples(SimpleNamespace(hospital_map=SimpleNamespace(resolution=.05)),path.tolist())
    assert len(sampled)==1+len(first.samples)+len(second.samples)
    assert np.allclose(path,np.array(sampled),atol=1e-14,rtol=0)


def test_dense_safety_sampling_enforces_translation_and_yaw_limits():
    edge=dubins_edge((0,0,0),(.8,.8,math.pi/2))
    dense=dense_interpolate(np.vstack((edge.start,edge.samples)))
    assert np.max(np.linalg.norm(np.diff(dense[:,:2],axis=0),axis=1))<=.025+1e-12
    dyaw=(np.diff(dense[:,2])+math.pi)%(2*math.pi)-math.pi
    assert np.max(np.abs(dyaw))<=math.radians(1)+1e-12


def test_full_padded_rectangle_detects_corner_collision_that_center_misses(tmp_path):
    world=ConstraintWorld.__new__(ConstraintWorld)
    occupancy=np.zeros((40,40),dtype=np.int8)
    world.map=CropMap(tmp_path/'map.yaml',tmp_path/'map.pgm',.05,(0.,0.,0.),40,40,occupancy,np.full((40,40),.01))
    world.map.full_origin=(0.,0.,0.)
    world.map.full_height=40
    world.map.row0=world.map.col0=0
    world.master=np.zeros((40,40),dtype=np.uint8)
    # Occupied square at x=[1.25,1.30], y=[1.20,1.25] intersects padded
    # rectangle at (1,1,0), but misses the unpadded 0.215 half-height.
    world.master[15,25]=254
    world.obstacle=world.master>=254
    world.grids={"allowed":np.ones((40,40),dtype=bool),"hard":np.zeros((40,40),dtype=bool)}
    world.pose_checks=world.exact_checks=0
    world.safe_threshold=.5
    assert not world.collision_free(np.array([[1.,1.,0.]]))
    assert world.collision_free(np.array([[.9,.9,0.]]))


def test_output_directories_and_json_are_write_once(tmp_path):
    out=fresh(tmp_path/'run')
    write_json(out/'protocol.json',{"x":1})
    with pytest.raises(FileExistsError):
        fresh(out)
    with pytest.raises(FileExistsError):
        write_json(out/'protocol.json',{"x":2})
