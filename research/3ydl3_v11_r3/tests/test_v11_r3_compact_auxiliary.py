from dataclasses import replace
import numpy as np
import pytest
from three_yd_v5.compact_auxiliary import StaticFromOccupancy,PackedHardMask,compact_owned_grid
from three_yd_v5.v9_storage import static_lut
from three_yd_v5.v9_maps import seal,seal_grid,immutable
from three_yd_v5.v9_r7_batch import field_binding
from arena_evaluation.semantic_costmap_composer import SemanticCostmap

@pytest.mark.parametrize('shape',[(1,1),(13,17),(15,16),(50,73),(102,129)])
@pytest.mark.parametrize('seed',range(4))
def test_all_values_and_native_crops_match_original(shape,seed):
 rng=np.random.default_rng(seed);occ=rng.integers(-128,128,shape,dtype=np.int8);hard=rng.random(shape)>.5
 static=StaticFromOccupancy(seal(occ));packed=PackedHardMask.from_array(hard)
 assert np.array_equal(np.asarray(static),static_lut(occ))
 assert np.array_equal(np.asarray(packed),hard)
 for idx in [(slice(None),slice(None)),(slice(1,-1),slice(3,-2)),(slice(None,None,-1),slice(None,None,2)),(0,0),(-1,-1),([0,-1],[0,-1]),(slice(9,2),slice(3,1)),(slice(None),slice(None,-1))]:
  assert np.array_equal(static[idx],static_lut(occ)[idx])
  assert np.array_equal(packed[idx],hard[idx])
 assert immutable(static.occupancy) and immutable(packed.packed)
 occ[:]=0;hard[:]=False
 assert np.array_equal(np.asarray(packed),np.unpackbits(packed.packed,axis=1,count=shape[1],bitorder='big').astype(bool))

def make_grid(shape=(1024,1025)):
 rng=np.random.default_rng(7);occ=rng.integers(-1,101,shape,dtype=np.int8);hard=occ>=65
 return SemanticCostmap(static_lut(occ),occ,np.zeros(shape,np.uint8),np.zeros(shape,np.float32),hard,np.ones(shape,bool),'p','s','r',True,{})

def test_global_fields_sealed_and_native_grid_keeps_exact_original_ack_inputs(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R8_MODE','OGR')
 g=make_grid();c=compact_owned_grid(g)
 assert type(c.internal_cost) is StaticFromOccupancy and type(c.hard_semantic_mask) is PackedHardMask
 assert c.internal_cost.occupancy is c.occupancy_grid
 sealed=seal_grid(c)
 assert np.array_equal(np.asarray(sealed.internal_cost),g.internal_cost)
 assert np.array_equal(np.asarray(sealed.hard_semantic_mask),g.hard_semantic_mask)
 assert np.array_equal(sealed.occupancy_grid,g.occupancy_grid)
 assert c.diagnostics['r11_auxiliary_stored_bytes']<c.diagnostics['r11_auxiliary_dense_bytes']/10
 old=field_binding(c.internal_cost);g.occupancy_grid[:]=42
 assert field_binding(c.internal_cost)==old
 for value in [c.internal_cost,c.hard_semantic_mask]:
  old=field_binding(value)
  if type(value) is StaticFromOccupancy:object.__setattr__(value,'occupancy',np.zeros(value.shape,np.int8))
  else:object.__setattr__(value,'packed',np.zeros_like(value.packed))
  assert field_binding(value)!=old

def test_custom_static_costs_and_small_grids_keep_original_storage():
 g=make_grid();g.internal_cost[100,100]^=1
 assert compact_owned_grid(g) is g
 small=make_grid((10,10));assert compact_owned_grid(small) is small

def test_dense_static_mode_preserves_custom_values_and_packs_only_hard():
 g=make_grid();g.internal_cost[100,100]^=1
 c=compact_owned_grid(g,derive_static=False)
 assert c.internal_cost is g.internal_cost and type(c.hard_semantic_mask) is PackedHardMask
 assert np.array_equal(np.asarray(c.hard_semantic_mask),g.hard_semantic_mask)
 assert c.diagnostics['r11_static_storage']=='original_dense'
