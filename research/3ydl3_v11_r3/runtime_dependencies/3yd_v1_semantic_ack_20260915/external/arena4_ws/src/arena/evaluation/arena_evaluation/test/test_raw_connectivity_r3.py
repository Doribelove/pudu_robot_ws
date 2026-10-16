from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from arena_evaluation.raw_connectivity_r3 import RawConnectivity


def map_(occ):
    h,w=occ.shape
    def cell(x,y):
        r,c=int(y),int(x)
        return (r,c) if 0<=r<h and 0<=c<w else None
    return SimpleNamespace(occupancy=occ,height=h,width=w,resolution=.05,
                          origin=(0.,0.,0.),world_to_cell=cell)


@pytest.mark.parametrize('seed',range(6))
@pytest.mark.parametrize('tile',[3,7,16])
def test_tiled_necessary_connectivity_matches_dense_all_pairs(seed,tile):
    rng=np.random.default_rng(seed)
    occ=np.where(rng.random((21,25))>.55,0,100).astype(np.int8)
    occ[2:4,4]=-1
    poses={str(i):(int(c),int(r),0.) for i,(r,c) in enumerate(
        zip(rng.integers(0,21,50),rng.integers(0,25,50)))}
    _,labels=cv2.connectedComponents((occ==0).astype(np.uint8),connectivity=8)
    graph=RawConnectivity(map_(occ),'map-hash',tile_cells=tile)
    graph.build(poses)
    for a in poses:
        for b in poses:
            x,y,_=poses[a];xx,yy,_=poses[b]
            cert=graph.certificate(a,b)
            expected=bool(labels[y,x] and labels[yy,xx] and labels[y,x]==labels[yy,xx])
            assert (cert['status']=='RAW_CONNECTED_SE2_UNPROVEN')==expected
    assert graph.stats['max_dense_tile_cells']<=tile**2


def test_four_tile_diagonal_corner_is_not_falsely_disconnected():
    occ=np.full((4,4),100,np.int8);occ[1,1]=occ[2,2]=0
    graph=RawConnectivity(map_(occ),'m',tile_cells=2)
    graph.build({'a':(1,1,0),'b':(2,2,0)})
    assert graph.certificate('a','b')['status']=='RAW_CONNECTED_SE2_UNPROVEN'


def test_wall_and_unknown_are_real_negative_certificates():
    occ=np.zeros((12,12),np.int8);occ[:,6]=-1
    graph=RawConnectivity(map_(occ),'m',tile_cells=4)
    graph.build({'left':(3,6,0),'right':(8,6,0),'unknown':(6,6,0)})
    assert graph.certificate('left','right')['status']=='PROVEN_RAW_FREE_DISCONNECTED'
    assert graph.certificate('left','unknown')['status']=='ENDPOINT_CENTER_NOT_FREE'


def test_certificate_hash_deterministic_and_bound_to_map():
    occ=np.zeros((10,10),np.int8);poses={'a':(1,1,0),'b':(8,8,0)}
    certs=[]
    for key in ['map1','map1','map2']:
        g=RawConnectivity(map_(occ),key,tile_cells=3);g.build(poses)
        certs.append(g.certificate('a','b'))
    assert certs[0]['sha256']==certs[1]['sha256']
    assert certs[0]['sha256']!=certs[2]['sha256']


def test_incomplete_or_resource_exhausted_scan_never_certifies():
    g=RawConnectivity(map_(np.zeros((8,8),np.int8)),'m',tile_cells=2,max_boundary_nodes=1)
    with pytest.raises(RuntimeError):g.certificate('a','b')
    with pytest.raises(MemoryError):g.build({'a':(0,0,0),'b':(7,7,0)})
    with pytest.raises(RuntimeError):g.certificate('a','b')
