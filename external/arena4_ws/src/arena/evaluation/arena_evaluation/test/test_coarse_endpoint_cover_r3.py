from collections import deque
import random
import time

import pytest

from arena_evaluation.coarse_endpoint_cover_r3 import anchor_cover
from arena_evaluation.tiled_topology_r3 import TopologyDeadlineExceeded


def graph(edges, isolated=()):
    result = {n: [] for n in isolated}
    for i, (a, b) in enumerate(edges):
        result.setdefault(a, []).append((b, i))
        result.setdefault(b, []).append((a, i))
    return result


def reachable(g, start, selected):
    visited = {start}
    queue = deque([start])
    while queue:
        for other, _ in g[queue.popleft()]:
            if other in selected and other not in visited:
                visited.add(other)
                queue.append(other)
    return visited


def test_safe_alternative_seed_keeps_required_far_detour():
    # Near seeds s1/g1 win the old forest; actual safe s2/g2 need x/y.
    s1, g1, s2, g2, x, y = [(i, 0, 1) for i in range(6)]
    g = graph([(s1,g1),(s2,x),(x,y),(y,g2),(s1,s2)])
    starts={s1:0.,s2:.1};goals={g1:0.,g2:.1}
    nodes, cert=anchor_cover(g,starts,goals)
    assert set(starts)|set(goals) <= reachable(g,s1,nodes)
    assert {x,y} <= nodes
    assert cert['groups'][0]['anchor']==g1


def test_disconnected_components_never_get_false_edges():
    a,b,c,d,e=[(i,0,1) for i in range(5)]
    g=graph([(a,b),(c,d)],isolated=[e])
    nodes,cert=anchor_cover(g,{a:0,c:0,e:0},{b:0,d:0})
    assert len(cert['groups'])==2 and e not in nodes
    assert d not in reachable(g,a,nodes)
    real={tuple(sorted((u,v))) for u in g for v,_ in g[u]}
    assert all(tuple(edge) in real for group in cert['groups'] for edge in group['edges'])


def test_no_shared_component_returns_no_route():
    a,b=(0,0,1),(0,1,1)
    nodes,cert=anchor_cover(graph([],isolated=[a,b]),{a:0},{b:0})
    assert not nodes and not cert['groups']


@pytest.mark.parametrize('seed',range(12))
def test_cover_matches_full_component_relation_for_every_endpoint(seed):
    rng=random.Random(seed)
    nodes=[(i//7,i%7,1) for i in range(70)]
    edges=[(a,b) for i,a in enumerate(nodes) for b in nodes[i+1:] if rng.random()<.025]
    g=graph(edges,isolated=nodes)
    starts={n:rng.random() for n in rng.sample(nodes,15)}
    goals={n:rng.random() for n in rng.sample(nodes,15)}
    selected,cert=anchor_cover(g,starts,goals)
    for a in starts:
        full=reachable(g,a,set(g))
        if full.intersection(goals):
            assert (full.intersection(starts)|full.intersection(goals)) <= reachable(g,a,selected)
    shuffled=list(g.items());rng.shuffle(shuffled)
    g2={n:list(reversed(adj)) for n,adj in shuffled}
    result,cert2=anchor_cover(g2,dict(reversed(list(starts.items()))),dict(reversed(list(goals.items()))))
    assert selected==result and cert==cert2


def test_deadline_prevents_search_acceptance():
    a=(0,0,1)
    with pytest.raises(TopologyDeadlineExceeded):
        anchor_cover({a:[]},{a:0},{a:0},deadline=time.monotonic()-1)
