from types import SimpleNamespace as S
import time
import numpy as np
import pytest
from three_yd_v5 import curve_l3
from three_yd_v5.r4_tiles import TiledSoft
from three_yd_v5.tailcost_occupancy import occupancy_on_free_cells
from three_yd_v5.v9_storage import occupancy_on_free_support


@pytest.mark.parametrize('rows',[1,7,96,256])
@pytest.mark.parametrize('density',[0.,.05,.5,1.])
def test_exact_occupancy_with_unknowns_overlaps_edges_and_mutations(rows,density):
    rng=np.random.default_rng(710)
    source=rng.integers(-128,128,(31,49),dtype=np.int8)
    hard=rng.random(source.shape)>density
    drive=rng.random(source.shape)>.2
    pieces=[((0,19,0,31),rng.uniform(-10,255,(19,31)).astype(np.float32)),
            ((13,31,20,49),rng.uniform(0,255,(18,29)).astype(np.float32))]
    soft=TiledSoft(source.shape,139.999998,drive,pieces)
    for _ in range(2):
        expected=occupancy_on_free_support(source,soft,hard,rows)
        actual=occupancy_on_free_cells(source,soft,hard,rows)
        assert actual.dtype==expected.dtype and actual.tobytes()==expected.tobytes()
        source[3,5]=-1;hard[3,5]=False;drive[3,5]=False
        pieces[-1][1][:]=73.25


def test_unknown_soft_layout_uses_original_converter():
    source=np.arange(256,dtype=np.uint8).view(np.int8).reshape(16,16)
    soft=np.arange(256,dtype=np.float32).reshape(16,16)
    hard=np.zeros((16,16),bool)
    assert np.array_equal(occupancy_on_free_cells(source,soft,hard),occupancy_on_free_support(source,soft,hard))


@pytest.mark.parametrize('seed_success',[False,True])
def test_early_detour_seed_retains_original_deadline_and_fallback(monkeypatch,seed_success):
    calls=[];deadline=time.monotonic()+10
    def solve(world,mask,guide,start,goal,until,cfg):
        calls.append((until,cfg))
        if cfg.get('r4_rounded_guide'):
            return (object() if seed_success else None),None,{'wall_ms':1.,'failure':'CURVE_NO_CERTIFIED_SOLUTION'}
        return object(),None,{'wall_ms':2.}
    monkeypatch.setattr(curve_l3,'_optimize_once',solve)
    p,_,log=curve_l3.optimize(S(),None,None,None,None,deadline,
        dict(curve_l3.CONFIG,r7_detour_seed=True,r4_failure_recovery=True))
    assert p is not None
    assert len(calls)==(1 if seed_success else 2)
    assert all(d<=deadline for d,c in calls)
    assert calls[0][1]['tracking_weight']==curve_l3.CONFIG['tracking_weight']*100
    if seed_success:assert log['r7_detour_seed']['selected']
    else:assert 'r7_detour_seed_failure' in log and not calls[1][1].get('r4_rounded_guide')


def test_seed_is_not_repeated_in_recovery_phase(monkeypatch):
    calls=[]
    def solve(*args):
        calls.append(args[-1]);return object(),None,{'wall_ms':1.}
    monkeypatch.setattr(curve_l3,'_optimize_once',solve)
    result=curve_l3.optimize(S(),None,None,None,None,time.monotonic()+10,
        dict(curve_l3.CONFIG,r7_detour_seed=True,r4_failure_recovery=True,r4_recovery_only=True))
    assert result[0] is not None and len(calls)==1
    assert calls[0]['tracking_weight']==curve_l3.CONFIG['tracking_weight']


@pytest.mark.parametrize('generic',[False,True])
def test_composition_preserves_policy_costmap_and_input_bytes(monkeypatch,generic):
    from three_yd_v5.fast_costmap import compose_bounded
    for name,value in [('THREEYD_R11_MODE','AB'),('THREEYD_V8_R4_MODE','B'),
                       ('THREEYD_V9_R1_MODE','S'),('THREEYD_V9_R8_MODE','OGR')]:
        monkeypatch.setenv(name,value)
    shape=(513,769);source=np.zeros(shape,np.int8);source[90:110,330:380]=100;source[300,550]=-1
    hard=source!=0;drivable=~hard;scope=np.zeros(shape,bool);scope[130:400,50:700]=True
    c=S(world=S(hard=hard,map=S(occupancy=source,resolution=.05),semantic=S(semantic_map_hash='same')),
        reference_cells=np.array([[151,80],[199,350],[370,650]]),global_drivable=drivable,
        plan=S(corridor_mask=drivable),dynamic=None,task={'signature':'same'},generic_mode=generic)
    inputs=[a.copy() for a in (source,hard,drivable,scope)]
    monkeypatch.setenv('THREEYD_V10_R7_MODE','control');old=compose_bounded(c,scope)
    monkeypatch.setenv('THREEYD_V10_R7_MODE','ABC');new=compose_bounded(c,scope)
    for name in ('internal_cost','occupancy_grid','expected_master_cost','hard_semantic_mask','affected_mask'):
        assert getattr(old,name).tobytes()==getattr(new,name).tobytes()
    assert old.policy_hash==new.policy_hash
    assert all(np.array_equal(a,b) for a,b in zip(inputs,(source,hard,drivable,scope)))


@pytest.mark.parametrize('cap',[0.,140.,-7.5])
def test_streamed_tile_freeze_keeps_bytes_and_avoids_later_copy(monkeypatch,cap):
    from three_yd_v5.r4_tiles import reference_soft
    from three_yd_v5.v9_r8_fields import freeze_array
    from three_yd_v5.v9_maps import immutable
    shape=(513,769);drive=np.ones(shape,bool);drive[10:20,40:50]=False
    before=drive.copy();ref=np.array([[20,30],[210,380],[480,700]])
    monkeypatch.setenv('THREEYD_V10_R7_MODE','ABC');old,_=reference_soft(shape,ref,.05,drive,cap)
    monkeypatch.setenv('THREEYD_V10_R7_MODE','ABCD');new,_=reference_soft(shape,ref,.05,drive,cap)
    assert np.asarray(old).tobytes()==np.asarray(new).tobytes()
    assert [x[0] for x in old.pieces]==[x[0] for x in new.pieces]
    for (_,a),(_,b) in zip(old.pieces,new.pieces):
        assert a.tobytes()==b.tobytes() and immutable(b)
        assert freeze_array(b) is b
        with pytest.raises(ValueError):b.flags.writeable=True
    assert np.array_equal(drive,before) and drive.flags.writeable
