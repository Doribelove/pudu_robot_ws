"""Route recovery only expands alternatives after a downstream rejection."""
from collections import defaultdict
from types import SimpleNamespace

from three_yd_v5 import recovery


class Cache(dict):
    def diagnostics(self):
        return {}


class Topology:
    def __init__(self):
        self.calls = []
        self.routes_list = [SimpleNamespace(signature='first', region_ids=(1, 2)),
                            SimpleNamespace(signature='second', region_ids=(1, 3))]
        self._last_first_phase_resume = False

    def routes(self, query, limit):
        self.calls.append(limit)
        self._last_first_phase_resume = limit > 1
        return self.routes_list[:limit]


class Controller:
    def __init__(self, world, topology, route, **kwargs):
        self.route = route
        self.timing = {}
        self.lifecycle = SimpleNamespace(clear=lambda: None)

    def plan_l3(self, query, session, auditor, spec):
        if self.route.signature == 'first':
            return {'success': False, 'failure_code': 'V5_L2_NO_PATH'}
        return {'success': True}


def make_runner(monkeypatch):
    caches = defaultdict(Cache)
    monkeypatch.setattr(recovery, 'get_cache', lambda views, name, **kwargs: caches[name])
    graph = Topology()
    runner = recovery.V5Runner(None, graph, None, None, controller_class=Controller)
    runner._query_key = lambda query: 'fixed-query'
    return runner, graph


def test_first_rejection_resumes_for_second_candidate(monkeypatch):
    monkeypatch.setenv('THREEYD_V6_LAZY_RESUME', '1')
    runner, graph = make_runner(monkeypatch)
    _, selected, output = runner.plan(SimpleNamespace(), SimpleNamespace(), None, None)
    assert output['success'] and selected.signature == 'second'
    assert graph.calls == [1, 3]
    assert output['candidate_recovery']['first_phase_resumed']
    assert [attempt['route_rank'] for attempt in output['candidate_recovery']['attempts']] == [1, 2]
    _, again, repeated = runner.plan(SimpleNamespace(), SimpleNamespace(), None, None)
    assert again.signature == 'second' and repeated['success']
    assert repeated['candidate_recovery']['preferred_route_hit']
    assert graph.calls == [1, 3, 3]


def test_first_success_skips_alternates(monkeypatch):
    monkeypatch.setenv('THREEYD_V6_LAZY_RESUME', '1')
    runner, graph = make_runner(monkeypatch)
    graph.routes_list = [graph.routes_list[1]]
    _, selected, output = runner.plan(SimpleNamespace(), SimpleNamespace(), None, None)
    assert output['success'] and selected.signature == 'second'
    assert graph.calls == [1]
    assert not output['candidate_recovery']['alternate_search_triggered']
    _, again, repeated = runner.plan(SimpleNamespace(), SimpleNamespace(), None, None)
    assert again.signature == 'second' and repeated['success']
    assert graph.calls == [1, 1]
    assert repeated['candidate_recovery']['lazy_alternates']
