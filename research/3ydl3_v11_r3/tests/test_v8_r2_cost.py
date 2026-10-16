"""Content-equivalence regressions for recovery cost changes."""
import hashlib
from types import SimpleNamespace
import numpy as np
import pytest
from arena_3d_v1.pipeline import corridor_dirty_transition as baseline
from three_yd_v5.dirty_transition import corridor_dirty_transition as optimized
from three_yd_v5.rectangle_bridge import RectanglePoseCertifier

@pytest.mark.parametrize('variant',['noop','opening','closing','mixed','empty','sliced','integer'])
def test_dirty_roi_is_content_exact(monkeypatch,variant):
    monkeypatch.setenv('THREEYD_V8_R2_MODE','B')
    rng=np.random.default_rng(822);old=rng.random((41,37))>.5;new=old.copy()
    if variant=='opening':new[:7,:]=True
    if variant=='closing':new[8:18,:]=False
    if variant=='mixed':new[::4,::3]^=True
    if variant=='empty':old=new=np.zeros((0,3),bool)
    if variant=='sliced':old=old[::-2,::2];new=new[::-2,::2]
    if variant=='integer':old=old.astype(np.int8)*7;new=new.astype(np.int8)*2
    assert optimized(old,new)==baseline(old,new)


def test_same_objects_changed_between_calls_are_not_reused(monkeypatch):
    monkeypatch.setenv('THREEYD_V8_R2_MODE','AB')
    old=np.zeros((7,8),bool);new=old.copy()
    initial=optimized(old,new);new[2,3]=True
    changed=optimized(old,new)
    assert changed==baseline(old,new)
    assert changed.target_hash!=initial.target_hash and changed.opened_cells==1
    old[2,3]=True
    assert optimized(old,new)==baseline(old,new)


def test_shape_guard_and_control_fallback(monkeypatch):
    monkeypatch.setenv('THREEYD_V8_R2_MODE','B')
    with pytest.raises(ValueError,match='identical shapes'):optimized(np.zeros((2,3)),np.zeros((3,2)))
    monkeypatch.setenv('THREEYD_V8_R2_MODE','control')
    assert optimized(np.eye(4),np.ones((4,4)))==baseline(np.eye(4),np.ones((4,4)))


def test_optimizations_are_not_acceptance_switches(monkeypatch):
    from three_yd_v5.r2_options import enabled,mode
    monkeypatch.setenv('THREEYD_V8_R2_MODE','A')
    assert enabled('A') and not enabled('B')
    monkeypatch.setenv('THREEYD_V8_R2_MODE','relax')
    with pytest.raises(ValueError):mode()


def test_ordering_only_prescreen_cannot_change_pose_certificates():
    # Production exact checker, all 24 yaws, all nine subpositions. Both
    # orders check the identical set, with the identical path-first preference.
    hard=np.zeros((36,36),bool);hard[16:20,17:20]=True
    world=SimpleNamespace(map=SimpleNamespace(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4))
    view=SimpleNamespace(safe=np.ones((12,12),bool),world_points=lambda r,c:np.column_stack(((c+.5)*.15,1.8-(r+.5)*.15)))
    bits=np.full((12,12),(1<<24)-1,np.uint32)
    cells=[(r,c) for r in range(12) for c in range(12)]
    def collect(order):
        cert=RectanglePoseCertifier(world,hard,view,bits)
        for cell,yaw in [((6,2),.2),((6,9),.2)]:cert.certify(cell,yaw)
        for cell in order:cert.certify(cell)
        return cert.cache
    a=collect(cells);b=collect(cells[::-1])
    assert a==b
    assert any(value is None for value in a.values()) and any(value is not None for value in a.values())
