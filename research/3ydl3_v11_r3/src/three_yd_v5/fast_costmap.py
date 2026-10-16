"""Compose the final bounded map once, preserving the pinned inflation bytes."""
import time,os
import numpy as np
import cv2
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_costmap_composer import SemanticCostmap, occupancy_to_static_layer, internal_soft_to_occupancy
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master
from .curve_semantic_binding import fresh_hash as fresh_semantic_hash
from .request_cost import request_digest as digest


def compose_bounded(controller,mask,cap=140):
    started=time.monotonic();w=controller.world;shape=w.hard.shape
    from .r11_options import enabled
    from .r11_fields import reference_soft,occupancy_chunked,static_chunked,bounded_inflation
    from .r4_options import enabled as r4_enabled
    if r4_enabled('B'):
        from .r4_tiles import reference_soft,bounded_inflation
    detail={};step=time.monotonic()
    if enabled('B'):
        soft,detail=reference_soft(shape,controller.reference_cells,w.map.resolution,controller.global_drivable,cap)
    else:
        seed=np.ones(shape,np.uint8)
        cv2.polylines(seed,[controller.reference_cells[:,::-1].astype(np.int32)],False,0,1)
        distance=cv2.distanceTransform(seed,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*w.map.resolution
        soft=(cap*np.minimum(1.,distance/2.)).astype(np.float32)
        soft[~controller.global_drivable]=200.
        del seed,distance
    detail['r11_soft_field_ms']=(time.monotonic()-step)*1000;step=time.monotonic()
    from .v9_review import enabled as review_enabled
    hard=None
    if review_enabled('S'):
        from .v9_storage import occupancy_on_free_support
        hard=w.hard|~controller.plan.corridor_mask
        from .v9_r8_options import enabled as r8_enabled
        if r8_enabled('O') and controller.generic_mode:
            from .v9_r8_fields import sparse_occupancy
            occupancy,extra=sparse_occupancy(w.map.occupancy,soft,hard)
            detail.update(extra)
        else:
            if 'B' in os.environ.get('THREEYD_V10_R7_MODE','control'):
                from .tailcost_occupancy import occupancy_on_free_cells
                occupancy=occupancy_on_free_cells(w.map.occupancy,soft,hard)
                detail['r7_free_cell_soft_conversion']=True
            else:
                occupancy=occupancy_on_free_support(w.map.occupancy,soft,hard)
            if r8_enabled('O'):
                detail.update(r8_sparse_occupancy_used=False,
                              r8_sparse_occupancy_fallback='semantic_map_original_conversion_selected')
    elif enabled('A'):occupancy=occupancy_chunked(w.map.occupancy,soft)
    else:
        occupancy=np.maximum(w.map.occupancy.astype(np.int16),internal_soft_to_occupancy(soft)).astype(np.int8)
        occupancy[w.map.occupancy<0]=-1
    detail['r11_occupancy_ms']=(time.monotonic()-step)*1000;step=time.monotonic()
    if hard is None:hard=w.hard|~controller.plan.corridor_mask
    occupancy[hard]=100
    version=0 if controller.dynamic is None else controller.dynamic.version
    reference_hash=digest(controller.reference_cells)
    key=(version,reference_hash,True,cap)
    # Preserve r8's policy identity as well as the final occupancy and master.
    base_policy=canonical_hash(['v4-reference-soft-v1',controller.task['signature'],key,digest(occupancy)])
    mask_hash=digest(mask)
    policy=canonical_hash([base_policy,mask_hash,'r5-bounded-l3-v1'])
    if 'C' in os.environ.get('THREEYD_V10_R7_MODE','control'):
        outside=~mask
        occupancy[outside]=100
        hard|=outside
        del outside
    else:
        occupancy[~mask]=100
        hard|=~mask
    if review_enabled('S'):
        from .v9_storage import static_lut
        static=static_lut(occupancy)
    else:static=static_chunked(occupancy) if enabled('A') else occupancy_to_static_layer(occupancy)
    detail['r11_binding_static_ms']=(time.monotonic()-step)*1000;step=time.monotonic()
    if enabled('B'):
        master,info=bounded_inflation(static,w.map.resolution);detail.update(info)
    else:
        master=pinned_nav2_effective_master(static,resolution=w.map.resolution,
            inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
    detail['r11_inflation_ms']=(time.monotonic()-step)*1000
    from .v9_r8_options import enabled as r8_enabled
    detail['r8_exact_tile_reduction_enabled']=r8_enabled('R')
    affected=np.broadcast_to(np.array(True),shape) if enabled('A') else np.ones(shape,bool)
    return SemanticCostmap(static,occupancy,master,soft,hard,affected,policy,
        fresh_semantic_hash(w.semantic),reference_hash,True,
        {**detail,'adapter_ms':(time.monotonic()-started)*1000,'dynamic_version':version,
         'finite_nonnegative':True,'max_soft_cost':float(soft.max()),'hard_mask_sha256':digest(w.hard),
         'search_bound_hash':mask_hash,'bounded_search_cells':int(mask.sum()),
         'search_roi_encoded_as_obstacle':True,'source_obstacles_unchanged':True,
         'soft_cost_preserved':True,'global_parking_transit_rule':True,
         'adapter':'r9 final bounded composition; one pinned inflation pass'})
