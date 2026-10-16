"""Runner integration with real canonical PathAudit and synthetic static maps."""

from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from arena_evaluation import two_layer_v1_r3_benchmark as runner
from arena_evaluation.endpoint_heading import sample_dubins
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.unified_four_backends_smoke import PlanResult


def _context(*, obstacle=None):
    occupancy = np.zeros((320, 400), dtype=np.int8)
    occupancy[[0, -1], :] = 100
    occupancy[:, [0, -1]] = 100
    if obstacle is not None:
        x, y, cost = obstacle
        occupancy[319 - int(y / .05), int(x / .05)] = cost
    hospital = HospitalMap(
        Path('/synthetic/map.yaml'), Path('/synthetic/map.pgm'), .05,
        (0., 0., 0.), 400, 320, occupancy,
        distance_transform_edt(occupancy == 0) * .05,
    )
    return SimpleNamespace(hospital_map=hospital, map_id='synthetic-r3', map_sha256='fixed-map')


def _query():
    return Query('A2B-01', [3., 6., 0.], [12., 6., 0.], seed=0)


def _dubins_points(query, *, steering=0.):
    sampled = sample_dubins(query.start, query.goal, radius_m=.8, spacing_m=.04)
    assert sampled is not None
    poses, _, _ = sampled
    return [dict(x=x, y=y, yaw=yaw, source='kinematic', motion_direction='forward',
                 steering=steering, planner_backend='nav2_smac_hybrid', backend_version='fake-transport')
            for x, y, yaw in poses]


def _route(query):
    return SimpleNamespace(polyline=[list(query.start[:2]), [6., 6.], [9., 6.], list(query.goal[:2])])


def _success(points, **diagnostics):
    return PlanResult(planner_success=True, points=deepcopy(points), failure_code='',
                      diagnostics={'planner_search_started': True,
                                   'costmap_update_acknowledged': True,
                                   'costmap_ack_mismatch_cells': 0, 'hard_mismatch': 0, 'soft_mismatch': 0,
                                   'stale_cells': 0, 'hash_mismatch': 0, 'sequence_mismatch': 0,
                                   'publication': {'request_id': 'measured:1:A2B-01'}, **diagnostics})


def _max_iterations(**diagnostics):
    return PlanResult(failure_code='ACTION_ABORTED', failure_detail='maximum iterations',
                      diagnostics={'planner_search_started': True,
                                   'failure_detail': 'maximum iterations',
                                   'expanded_states': 1000000, 'generated_states': 1200000,
                                   **diagnostics})


def _ack_failure():
    return PlanResult(failure_code='EXACT_ACK_FAILED_CLOSED', failure_detail='one stale cell',
                      diagnostics={'planner_search_started': False,
                                   'costmap_update_acknowledged': False,
                                   'costmap_ack_mismatch_cells': 1,
                                   'failure_detail': 'one stale cell',
                                   'server_costmap_content_hash': 'stale-server'})


class _Clock:
    def __init__(self):
        self.value = 100.

    def monotonic(self):
        return self.value

    def process_time(self):
        return self.value


class _Session:
    def __init__(self, responses, clock, durations=None):
        self.responses = responses
        self.clock = clock
        self.durations = durations or [.1] * len(responses)
        self.starts = []
        self.calls = []

    def begin_request(self, request_id, deadline):
        self.starts.append((request_id, deadline))
        self.deadline = deadline

    def plan(self, query, spec, **kwargs):
        index = len(self.calls)
        self.calls.append({'query': deepcopy(query), 'deadline': self.deadline,
                           'time': self.clock.monotonic(), 'mask': kwargs['allowed_mask'].copy()})
        self.clock.value += self.durations[index]
        value = self.responses[index]
        if isinstance(value, Exception):
            raise value
        return deepcopy(value)


def _run(monkeypatch, ctx, responses, *, query=None, route=None, budget=1., durations=None,
         selector_delay=.02, selector_failure=None):
    query = query or _query()
    route = route or _route(query)
    clock = _Clock()
    monkeypatch.setattr(runner, 'time', clock)
    session = _Session(responses, clock, durations)
    selected = []

    def selector(topology, actual_query, *, timing, deadline):
        selected.append((topology, actual_query, deadline))
        clock.value += selector_delay
        timing.update(start_candidate_count=2, goal_candidate_count=3)
        if isinstance(selector_failure, Exception):
            raise selector_failure
        return None, None, None if selector_failure else route, selector_failure or ''

    result = runner.run_query(ctx, query, 'synthetic-topology', selector, session,
                              SimpleNamespace(backend='fake-smac', version='test'),
                              PathAuditor(ctx, source_commit='test-r3'), 'measured:1:A2B-01', budget)
    return result, session, selected


def test_real_dubins_straight_success_passes_canonical_audit(monkeypatch):
    ctx = _context()
    query = _query()
    row, session, _ = _run(monkeypatch, ctx, [_success(_dubins_points(query))], query=query)
    assert row['action_success'] is True
    assert row['final_valid_success'] is True
    assert row['static_footprint_valid'] is True
    assert row['kinematic_valid'] is True
    assert row['canonical_path_within_mask'] is True
    assert row['reverse_distance_m'] == row['in_place_rotation_count'] == 0
    assert row['maximum_curvature'] <= 2.5 + .001
    assert row['costmap_ack_mismatch_cells'] == 0
    assert row['canonical_sampled_pose_count'] > len(row['points'])
    assert row['canonical_path_hash'] and row['canonical_pose_hash'] and row['canonical_mask_hash']
    assert row['failure_code'] == ''
    assert row['l3_calls'] == len(session.calls) == 1


def test_real_dubins_constant_radius_turn_passes_canonical_audit(monkeypatch):
    ctx = _context()
    query = Query('A2B-01', [6., 6., 0.], [6.8, 6.8, math.pi / 2])
    points = _dubins_points(query, steering=math.atan(.5 / .8))
    route = SimpleNamespace(polyline=[[p['x'], p['y']] for p in points])
    row, _, _ = _run(monkeypatch, ctx, [_success(points)], query=query, route=route)
    assert row['final_valid_success'] is True
    assert row['static_footprint_valid'] is True
    assert row['kinematic_valid'] is True
    assert row['maximum_curvature'] <= 1.25 + .001
    assert row['reverse_distance_m'] == row['in_place_rotation_count'] == 0


@pytest.mark.parametrize('cost', [100, -1])
def test_collision_and_unknown_paths_cannot_be_final_valid(monkeypatch, cost):
    ctx = _context(obstacle=(7., 6., cost))
    row, session, _ = _run(monkeypatch, ctx, [_success(_dubins_points(_query()))])
    assert row['action_success'] is True
    assert row['final_valid_success'] is False
    assert row['static_footprint_valid'] is False
    assert row['failure_code'] == 'STATIC_FOOTPRINT_COLLISION'
    assert row['canonical_exact_footprint_check_count'] > 0
    assert len(session.calls) == 1
    assert row['points'], 'Rejected candidate path must remain available as evidence'


@pytest.mark.parametrize('violation', ['reverse', 'rotation', 'discontinuity'])
def test_kinematic_invalid_success_is_rejected_and_preserved(monkeypatch, violation):
    points = _dubins_points(_query())
    if violation == 'reverse':
        points[20]['motion_direction'] = 'reverse'
        failure = 'REVERSE_MOTION'
    elif violation == 'rotation':
        rotated = deepcopy(points[20])
        rotated['yaw'] += .1
        points.insert(21, rotated)
        failure = 'IN_PLACE_ROTATION_FORBIDDEN'
    else:
        del points[20:40]
        failure = 'POSITION_DISCONTINUITY'
    row, session, _ = _run(monkeypatch, _context(), [_success(points)])
    assert row['action_success'] is True
    assert row['final_valid_success'] is False
    assert row['kinematic_valid'] is False
    assert row['failure_code'] == failure
    assert row['points'] and len(session.calls) == 1


def test_ack_failure_never_retries_or_starts_planner(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [_ack_failure()])
    assert row['final_valid_success'] is False
    assert row['planner_search_started'] is False
    assert row['failure_code'] == 'EXACT_ACK_FAILED_CLOSED'
    assert row['l3_calls'] == 0
    assert len(session.calls) == len(row['attempts']) == 1
    assert row['attempts'][0]['costmap_ack_mismatch_cells'] == 1
    assert row['attempts'][0]['server_costmap_content_hash'] == 'stale-server'


def test_corridor_includes_connectors_and_preserves_obstacles():
    ctx = _context(obstacle=(9., 6., -1))
    # First and last segments represent virtual endpoint connectors.
    route = SimpleNamespace(polyline=[[3., 6.], [4., 6.], [8., 6.], [8., 10.], [12., 10.]])
    mask, info = runner.corridor(ctx, route)
    expanded, expanded_info = runner.corridor(ctx, route, expansion=True)
    for x, y in ([3., 6.], [3.5, 6.], [8., 8.], [11., 10.], [12., 10.]):
        assert mask[ctx.hospital_map.world_to_cell(x, y)]
    assert not np.any(mask & (ctx.hospital_map.occupancy != 0))
    assert not np.any(expanded & (ctx.hospital_map.occupancy != 0))
    assert np.all(~mask | expanded)
    assert np.count_nonzero(expanded) > np.count_nonzero(mask)
    assert info['corner_count'] > 0
    assert info['corridor_expansion'] is False
    assert expanded_info['corridor_expansion'] is True


def test_packed_corridor_cache_matches_cold_mask_and_cannot_be_mutated():
    ctx=_context();route=_route(_query());cache=runner.CorridorCache(max_bytes=320*400//8)
    first,info=cache.get(ctx,route)
    assert info['corridor_cache_hit'] is False
    saved=first.copy();first[:]=False
    second,info=cache.get(ctx,route)
    assert info['corridor_cache_hit'] is True and np.array_equal(second,saved)
    expanded,info=cache.get(ctx,route,expansion=True)
    assert info['corridor_cache_hit'] is False and cache.bytes<=cache.max_bytes
    _,info=cache.get(ctx,route)
    assert info['corridor_cache_hit'] is False, 'The byte cap must evict the previous binding'
    ctx.map_sha256='different-map';_,info=cache.get(ctx,route)
    assert info['corridor_cache_hit'] is False
    route.polyline[-1][0]-=1.;_,info=cache.get(ctx,route)
    assert info['corridor_cache_hit'] is False


def test_retry_uses_same_deadline_and_retains_primary_failure(monkeypatch):
    query = _query()
    row, session, selected = _run(
        monkeypatch, _context(), [_max_iterations(), _success(_dubins_points(query))],
        query=query, durations=[.25, .15], budget=1.,
    )
    assert row['final_valid_success'] is True
    assert row['primary_failure'] == 'SMAC_MAX_ITERATIONS'
    assert row['failure_code'] == ''
    assert row['l3_calls'] == len(session.calls) == len(row['attempts']) == 2
    assert len(session.starts) == 1
    assert selected[0][2] == session.starts[0][1] == session.calls[0]['deadline'] == session.calls[1]['deadline']
    first, second = row['attempts']
    assert first['failure_code'] == 'SMAC_MAX_ITERATIONS'
    assert first['expanded_states'] == 1000000
    assert second['retry_reason'] == 'SMAC_MAX_ITERATIONS'
    assert first['corridor_expansion'] is False and second['corridor_expansion'] is True
    assert second['remaining_before_s'] < first['remaining_before_s']
    assert row['fallback_used'] is False


def test_retry_is_bounded_to_one_corridor_expansion(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [_max_iterations(), _max_iterations()])
    assert row['final_valid_success'] is False
    assert row['primary_failure'] == row['failure_code'] == 'SMAC_MAX_ITERATIONS'
    assert len(session.calls) == len(row['attempts']) == 2
    assert [a['corridor_expansion'] for a in row['attempts']] == [False, True]
    assert all(a['expanded_states'] == 1000000 for a in row['attempts'])


def test_retry_ack_failure_preserves_first_search_failure(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [_max_iterations(), _ack_failure()])
    assert row['final_valid_success'] is False
    assert row['primary_failure'] == 'SMAC_MAX_ITERATIONS'
    assert row['failure_code'] == 'EXACT_ACK_FAILED_CLOSED'
    assert row['l3_calls'] == 1
    assert len(session.calls) == 2
    assert row['attempts'][0]['planner_search_started'] is True
    assert row['attempts'][1]['planner_search_started'] is False


def test_deadline_exhaustion_prevents_retry(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [_max_iterations()], durations=[1.1], budget=1.)
    assert row['final_valid_success'] is False
    assert row['failure_code'] == 'REQUEST_DEADLINE'
    assert len(session.calls) == 1
    assert row['remaining_budget_s'] == 0.


def test_endpoint_stage_uses_same_budget_and_prevents_expired_search(monkeypatch):
    row, session, selected = _run(monkeypatch, _context(), [], selector_delay=1.1, budget=1.)
    assert row['final_valid_success'] is False
    assert row['failure_code'] == 'REQUEST_DEADLINE'
    assert selected[0][2] == session.starts[0][1]
    assert session.calls == []
    assert row['l3_calls'] == 0


def test_endpoint_failure_retains_detailed_reason_and_never_calls_smac(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [], selector_failure='ENDPOINT_LOCAL_SE2_NO_PATH')
    assert row['failure_code'] == 'ENDPOINT_LOCAL_SE2_NO_PATH'
    assert row['compatibility_failure_group'] == 'L1_ENDPOINT_NOT_ATTACHABLE'
    assert row['start_candidate_count'] == 2 and row['goal_candidate_count'] == 3
    assert row['final_valid_success'] is False
    assert row['planner_search_started'] is False
    assert session.calls == []


def test_backend_diagnostics_cannot_override_canonical_collision_verdict(monkeypatch):
    ctx = _context(obstacle=(7., 6., 100))
    response = _success(_dubins_points(_query()), final_valid_success=True,
                        static_footprint_valid=True, kinematic_valid=True, failure_code='')
    row, _, _ = _run(monkeypatch, ctx, [response])
    assert row['final_valid_success'] is False
    assert row['static_footprint_valid'] is False
    assert row['failure_code'] == 'STATIC_FOOTPRINT_COLLISION'


def test_session_exception_returns_complete_failed_row(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [RuntimeError('injected ACK serialization failure')])
    assert row['final_valid_success'] is False
    assert row['failure_code']
    assert row['planner_search_started'] is False
    assert 'injected ACK serialization failure' in str(row)
    assert row['request_id'] == 'measured:1:A2B-01'
    assert row['query_id'] == 'A2B-01'
    assert len(session.calls) == 1
    assert len(row['attempts']) == 1


def test_selector_exception_returns_complete_failed_row(monkeypatch):
    row, session, _ = _run(monkeypatch, _context(), [],
                           selector_failure=RuntimeError('injected connector cache failure'))
    assert row['final_valid_success'] is False
    assert row['failure_code']
    assert row['planner_search_started'] is False
    assert 'injected connector cache failure' in str(row)
    assert row['request_id'] == 'measured:1:A2B-01'
    assert row['query_id'] == 'A2B-01'
    assert session.calls == []


def test_runner_rejects_inconsistent_ack_true_with_nonzero_exact_mismatch(monkeypatch):
    response = _success(_dubins_points(_query()), costmap_ack_mismatch_cells=1,
                          costmap_update_acknowledged=True)
    row, _, _ = _run(monkeypatch, _context(), [response])
    print('final_valid', row['final_valid_success'], 'ack', row['costmap_update_acknowledged'],
          'exact_mismatch', row['costmap_ack_mismatch_cells'])
    assert row['final_valid_success'] is False


def test_backend_failure_code_cannot_erase_canonical_attempt_failure(monkeypatch):
    response = _success(_dubins_points(_query()), failure_code='')
    row, _, _ = _run(monkeypatch, _context(obstacle=(7., 6., 100)), [response])
    print('row_failure', row['failure_code'], 'attempt_failure', row['attempts'][0]['failure_code'])
    assert row['final_valid_success'] is False
    assert row['failure_code'] == 'STATIC_FOOTPRINT_COLLISION'
    assert row['attempts'][0]['failure_code'] == 'STATIC_FOOTPRINT_COLLISION'


def test_malformed_backend_path_keeps_complete_failed_request(monkeypatch):
    points = _dubins_points(_query())
    points[0].pop('yaw')
    row, _, _ = _run(monkeypatch, _context(), [_success(points)])
    assert row['final_valid_success'] is False
    assert row['query_id'] == 'A2B-01'
    assert row['failure_code']
    assert len(row['attempts']) == 1


def test_safe_saved_poses_with_unsafe_smoothed_interpolation_are_rejected(monkeypatch):
    # Translated real failure geometry; two saved endpoints are safe, but the
    # interpolated polygon violates the frozen cell-diagonal guard by .3 mm.
    ctx=_context(obstacle=(2.875,4.075,100))
    points=[{'x': 2.629539675579629, 'y': 3.9463450048715316, 'yaw': 1.6301316336655125, 'source': 'kinematic', 'motion_direction': 'forward', 'steering': -0.36295522402114927, 'planner_backend': 'Nav2 SmacPlannerHybrid DUBIN', 'backend_version': '1.1.20'}, {'x': 2.6236330662393854, 'y': 4.045774438476954, 'yaw': 1.5544757655217856, 'source': 'kinematic', 'motion_direction': 'forward', 'steering': -0.3666950468144754, 'planner_backend': 'Nav2 SmacPlannerHybrid DUBIN', 'backend_version': '1.1.20'}]
    for pose in points:
        assert not ctx.hospital_map.footprint_collision((pose['x'],pose['y'],pose['yaw']),runner.legacy.FOOTPRINT)
    query=Query('A2B-01',[points[0][k] for k in ('x','y','yaw')],[points[-1][k] for k in ('x','y','yaw')])
    route=SimpleNamespace(polyline=[[p['x'],p['y']] for p in points])
    row,session,_=_run(monkeypatch,ctx,[_success(points)],query=query,route=route)
    assert row['action_success'] is True
    assert row['final_valid_success'] is False
    assert row['static_footprint_valid'] is False
    assert row['failure_code']=='STATIC_FOOTPRINT_COLLISION'
    expected=[{**p,'source_commit':'test-r3'} for p in points]
    path_hash=runner.legacy._path_hash(expected)
    expected=[{**p,'path_hash':path_hash} for p in expected]
    assert len(session.calls)==1 and row['points']==expected


def _lazy_preparation(monkeypatch, query, clock, delays=(.2,.15,.1), capacity=32):
    class Tile:
        key='fixed-tile-map-config-source-binding'
        stats={'tile_refine_wall_ms':0.,'topology_load_wall_ms':0.,
               'tile_cache_hit_count':0,'tile_cache_miss_count':0}
        calls=[]
        def candidate_tiles(self,start,goal,*,deadline):
            self.calls.append(('coarse',deadline));clock.value+=delays[0]
            return [(0,0)]
        def artifact(self,selected,*,deadline):
            self.calls.append(('artifact',deadline));clock.value+=delays[1]
            self.stats['tile_refine_wall_ms']+=delays[1]*1000
            return SimpleNamespace(metadata={'tiles':selected})
    class Selector:
        def __init__(self,topology,footprint):
            clock.value+=delays[2];self.binding={'topology':'fixed-graph'}
            self.last_certificate=None
        def __call__(self,topology,actual,*,timing,deadline):
            tile.calls.append(('connector',deadline));clock.value+=.05
            self.last_certificate={'query_id':actual.query_id}
            return None,None,_route(actual),''
    tile=Tile();tile.stats=dict(Tile.stats);tile.calls=[]
    monkeypatch.setattr(runner,'ReachableEndpointSelector',Selector)
    return tile,runner.QueryTopologyPreparer(tile,runner.legacy.FOOTPRINT,capacity)


def test_query_dependent_topology_preparation_is_inside_shared_request_budget(monkeypatch):
    ctx=_context();query=_query();clock=_Clock();monkeypatch.setattr(runner,'time',clock)
    tile,preparer=_lazy_preparation(monkeypatch,query,clock)
    session=_Session([_success(_dubins_points(query))],clock)
    row=runner.run_query(ctx,query,None,None,session,SimpleNamespace(backend='fake',version='test'),
                         PathAuditor(ctx,source_commit='test'), 'measured:1:A2B-01',1.,preparer=preparer)
    assert row['final_valid_success'] is True
    assert row['query_topology_prepare_wall_ms']==pytest.approx(450.)
    assert row['online_wall_ms']==pytest.approx(600.)
    assert row['remaining_budget_s']==pytest.approx(.4)
    assert [name for name,_ in tile.calls]==['coarse','artifact','connector']
    assert all(deadline==101. for _,deadline in tile.calls)
    assert session.calls[0]['time']==pytest.approx(100.5)
    assert preparer.last_preparation['complete'] is True


@pytest.mark.parametrize('delays,completed_stages', [((.6,0.,0.),['coarse']),
                          ((.1,.5,0.),['coarse','artifact']),((.1,.1,.4),['coarse','artifact'])])
def test_topology_or_index_deadline_failure_never_starts_connector_or_smac(monkeypatch,delays,completed_stages):
    ctx=_context();query=_query();clock=_Clock();monkeypatch.setattr(runner,'time',clock)
    tile,preparer=_lazy_preparation(monkeypatch,query,clock,delays)
    session=_Session([],clock)
    row=runner.run_query(ctx,query,None,None,session,SimpleNamespace(backend='fake',version='test'),
                         PathAuditor(ctx,source_commit='test'),'measured:1:A2B-01',.5,preparer=preparer)
    assert row['failure_code']=='TOPOLOGY_REQUEST_DEADLINE'
    assert row['final_valid_success'] is False and row['planner_search_started'] is False
    assert row['online_wall_ms']>=600.-1e-6
    assert row['remaining_budget_s']==0
    assert [name for name,_ in tile.calls]==completed_stages
    assert not session.calls and not preparer.entries
    assert preparer.last_certificate is None
    assert preparer.last_preparation['complete'] is False


def test_lazy_topology_cache_binds_poses_and_tile_identity_and_is_bounded(monkeypatch):
    query=_query();clock=_Clock();monkeypatch.setattr(runner,'time',clock)
    tile,preparer=_lazy_preparation(monkeypatch,query,clock,capacity=1)
    first=preparer.resolve(query,110.,{})
    timing={};assert preparer.resolve(query,110.,timing)==first
    assert timing['query_topology_cache_hit'] is True
    assert len(tile.calls)==2
    moved=deepcopy(query);moved.goal[0]+=.1
    timing={};preparer.resolve(moved,110.,timing)
    assert timing['query_topology_cache_hit'] is False and len(preparer.entries)==1
    timing={};preparer.resolve(query,110.,timing)
    assert timing['query_topology_cache_hit'] is False, 'previous pose entry was evicted'
    tile.key='changed-map-or-configuration-or-implementation'
    timing={};preparer.resolve(query,110.,timing)
    assert timing['query_topology_cache_hit'] is False


def test_cold_selects_single_fixed_query_before_preparation_without_mutating_inputs():
    queries=[_query() for _ in range(20)]
    selected=runner.queries_for_stage(queries,'cold')
    assert len(selected)==1 and selected[0] is queries[0]
    assert len(queries)==20
    assert runner.queries_for_stage(queries,'formal') is queries
    assert runner.queries_for_stage(queries,'smoke') is queries
