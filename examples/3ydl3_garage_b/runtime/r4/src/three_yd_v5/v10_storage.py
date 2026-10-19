"""V10 storage ownership: identical current cells, no cached audit decisions."""
import os
import numpy as np
from .v9_maps import seal, immutable

def enabled():
    return os.environ.get('THREEYD_V10_STORAGE', '1') == '1'

def prepare(w):
    if not enabled(): return
    from .tail_filehash import prepare as prepare_filehash
    prepare_filehash(w.map)
    from .v9_maps import prepare_world
    prepare_world(w)
    # Before topology/views retain references. Collision ABI is read-only uint8.
    w.hard=seal(w.hard)
    from arena_3d_v1.semantic_world import SweptChecker
    checker=SweptChecker(w.hard.view(np.uint8),w.map.resolution,w.map.origin,*w.vehicle[:2])
    if getattr(w,'v7_map_only',False): w._checker=checker
    else: w.checker=checker

def shared_static(w):
    return w.hard if enabled() and immutable(w.hard) else None

def prepare_topology(topology):
    """Seal newly constructed static labels before the loader exposes them.

    This is map storage only: no task, route, query, or content digest is
    computed here. The first content hash still occurs in the actual request.
    Unknown topology APIs retain their original mutable representation.
    """
    from .topology import V5Topology
    from .topology_astar import AStarTopology
    if not enabled() or type(topology) not in (V5Topology, AStarTopology):
        return False
    a = topology.subregions
    if type(a) is not np.ndarray or a.ndim != 2 or a.dtype != np.int16:
        return False
    topology.subregions = seal(a)
    return True

def collision_checker(hard,world):
    from arena_3d_v1.semantic_world import SweptChecker
    # Temporary request-local bool is not mutated while the checker uses it.
    a=np.asarray(hard)
    if a.dtype==np.dtype(bool) and a.flags.c_contiguous: a=a.view(np.uint8)
    return SweptChecker(a,world.map.resolution,world.map.origin,*world.vehicle[:2])


def empty_overlay(shape):
    return np.broadcast_to(np.frombuffer(bytes(1),dtype=bool),shape)

def retained_mask(mask):
    return seal(mask) if enabled() else mask

def initial_scope(priority,generic):
    if enabled() and generic is True:
        value=seal(priority)
        return value,value
    from .v9_r9_buffers import initial_search_scope
    return initial_search_scope(priority,generic)
