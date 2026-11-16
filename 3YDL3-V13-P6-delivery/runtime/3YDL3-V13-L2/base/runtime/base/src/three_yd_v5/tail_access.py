"""Reuse an identical already-built mask when no parking binding is needed."""
from dataclasses import replace
from .tail_options import enabled
from .v9_maps import content_digest,seal


def _metadata(c):
    if c.generic_mode or c.traffic.endpoint_parking or c.r13_semantic_seam is not None:return None
    regions=c.topology.regions
    if any(regions[r]['kind'] not in ('lane','junction_area') for r in c.topology_route.region_ids):return None
    access=c.topology.contexts[c.topology_route.signature]['endpoint_access']
    if any(regions[int(access[k]['semantic_region_id'])]['kind']=='parking_area' for k in ('start','goal')):return None
    return (tuple(c.topology_route.region_ids),tuple(sorted((r,v['kind']) for r,v in regions.items())),
            tuple((k,int(access[k]['semantic_region_id']),int(access[k]['road_id'])) for k in ('start','goal')),
            float(c.world.map.resolution))


def _binding(c):
    meta=_metadata(c)
    if meta is None:return None
    arrays=(c.topology.subregions,c.world.hard,c.global_drivable,c.plan.corridor_mask)
    return meta+tuple((a.shape,a.dtype.str,content_digest(a)) for a in arrays)


def remember(c):
    if not enabled('P') or _metadata(c) is None:return
    # Constructor ownership is exclusive here. Later immutable storage sealing
    # already required this same copy; doing it here enables safe digest reuse.
    c.global_drivable=seal(c.global_drivable)
    c._r4_access_binding=_binding(c)


def reuse(c):
    if not enabled('P') or getattr(c,'_r4_access_binding',None) is None:return False
    if _binding(c)!=c._r4_access_binding:return False
    # With no parking regions, access is identically false; selected and all
    # lane/junction IDs are exactly those used by the constructor. Its .45 m
    # dilation and current hard-mask intersection are therefore unchanged.
    c.plan=replace(c.plan,diagnostics={**dict(c.plan.diagnostics),
        'endpoint_parking_access_tube_m':1.2,'whole_endpoint_parking_transit':False,'endpoint_access':{}})
    c.server_l3_mask=c.plan.corridor_mask;c.pending_l3_mask=c.plan.corridor_mask;c._pending_l3_hash=''
    c.timing['r4_identical_nonparking_access_reused']=True
    return True
