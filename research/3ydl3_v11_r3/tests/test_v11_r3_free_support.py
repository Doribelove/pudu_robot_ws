"""Independent frozen-G oracle for every pre-bound occupancy byte."""
from pathlib import Path
import importlib.util,hashlib,json
import numpy as np
import pytest
from three_yd_v5.r4_tiles import TiledSoft
from three_yd_v5.v9_r8_fields import sparse_occupancy
p=Path(__file__).parent/'oracles/v11_r3_g_fields.py'
assert hashlib.sha256(p.read_bytes()).hexdigest()==json.loads(p.with_name('v11_r3_g_fields_source.json').read_text())['sha256']
spec=importlib.util.spec_from_file_location('three_yd_v5._frozen_r3_g_fields',p)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

@pytest.mark.parametrize('seed',range(8))
@pytest.mark.parametrize('fraction',[0.,.05,.5,1.])
def test_all_prebound_cells_preserve_unknown_hard_background_and_overlaps(seed,fraction):
 rng=np.random.default_rng(seed);shape=(57,83)
 source=rng.integers(-128,128,shape,dtype=np.int8)
 hard=rng.random(shape)>=fraction;drivable=rng.random(shape)>.3
 pieces=[((3,31,5,65),rng.uniform(-50,260,(28,60)).astype(np.float32)),
         ((17,53,21,79),rng.uniform(-50,260,(36,58)).astype(np.float32))]
 soft=TiledSoft(shape,float(rng.uniform(0,200)),drivable,pieces)
 expected,_=module.sparse_occupancy(source,soft,hard,rows=13)
 actual,detail=sparse_occupancy(source,soft,hard,rows=13)
 assert np.array_equal(actual,expected)
 assert detail['r11_sparse_background_free_cells']==np.count_nonzero(~hard)
 assert np.all(actual[hard]==100)

@pytest.mark.parametrize('layout',['reverse','stride'])
def test_noncontiguous_original_inputs_preserve_all_bytes(layout):
 rng=np.random.default_rng(14);raw=rng.integers(-128,128,(53,87),dtype=np.int8)
 raw=raw[::-1,::-1] if layout=='reverse' else raw[::2,::3]
 hard=rng.random(raw.shape)>.1;soft=TiledSoft(raw.shape,123.00001,rng.random(raw.shape)>.3,[])
 assert np.array_equal(sparse_occupancy(raw,soft,hard)[0],module.sparse_occupancy(raw,soft,hard)[0])
