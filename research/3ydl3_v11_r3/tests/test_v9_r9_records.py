from copy import deepcopy
from dataclasses import dataclass
import numpy as np
import pytest
from three_yd_v5.v9_r9_records import InternalPoints, pack_internal
from three_yd_v5.r6_cache import resident_bytes


def rows(n=2000):
    return [{'x': float(i), 'y': -0.0, 'yaw': .3, 'source': 'original',
             'integer': 2**60 + i, 'flag': bool(i % 2), 'none': None} for i in range(n)]


def test_internal_storage_preserves_exact_bits_types_and_independent_delivery(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    original = rows()
    packed = pack_internal(original)
    assert type(packed) is InternalPoints
    assert resident_bytes(packed) < resident_bytes(original) / 3
    original[0]['x'] = -5
    a, b = deepcopy(packed), deepcopy(packed)
    assert type(a) is list and all(type(row) is dict for row in a)
    assert a == b == rows()
    assert np.array([v['y'] for v in a], np.float64).tobytes() == np.array([v['y'] for v in rows()], np.float64).tobytes()
    assert type(a[0]['integer']) is int and type(a[0]['flag']) is bool
    a[0]['x'] = 123
    a.append({'different': True})
    assert deepcopy(packed) == b


def test_deepcopy_retains_shared_container_identity_and_metadata_isolation(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    packed = pack_internal(rows(4))
    record = {'points': packed, 'alias': packed, 'metadata': {'nested': [1]}}
    copied = deepcopy(record)
    assert copied['points'] is copied['alias'] and copied['points'] is not packed
    copied['metadata']['nested'].append(2)
    assert record['metadata']['nested'] == [1]


@pytest.mark.parametrize('unknown', [[], [{'x': 1., 'nested': {'v': 1}}],
                                    [{'x': 1.}, {'different': 2.}], [{'x': np.float64(1.)}]])
def test_unknown_schema_uses_exact_original_storage(monkeypatch, unknown):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    assert pack_internal(unknown) is unknown


def test_shared_row_and_subclass_semantics_are_not_reencoded(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    row = {'x': 1.}
    repeated = [row, row]
    assert pack_internal(repeated) is repeated
    class Custom(dict):
        pass
    custom = [Custom(x=1.)]
    assert pack_internal(custom) is custom


def test_disabled_mode_keeps_original_and_invalid_mode_rejects(monkeypatch):
    original = rows(3)
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'control')
    assert pack_internal(original) is original
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'unexpected')
    with pytest.raises(ValueError):
        pack_internal(original)


def test_record_storage_does_not_change_public_result_or_metadata(monkeypatch):
    from three_yd_v5.fullroute import record_from
    @dataclass
    class Result:
        points: list
        diagnostics: dict
        path_audit: object = None
        expanded_states: int = 0
        generated_states: int = 0
        first_solution_time_ms: object = None
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    public = Result(rows(5), {'original_clock': 3})
    record = record_from({'success': True, 'result': public, 'bounded_l3': {'limit': 400000}})
    assert type(public.points) is list and public.diagnostics == {'original_clock': 3}
    assert type(record['result'].points) is InternalPoints
    delivered = deepcopy(record)
    assert delivered['result'].points == public.points and type(delivered['result'].points) is list
    delivered['result'].points[0]['x'] = 7
    assert deepcopy(record)['result'].points[0]['x'] == 0


def test_whole_cache_keeps_compact_storage_and_detaches_every_mutable_delivery(monkeypatch):
    from three_yd_v5.fullroute import RouteCache
    from types import SimpleNamespace
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    monkeypatch.setenv('THREEYD_V9_R1_MODE', 'S')
    data = rows()
    value = {'result': SimpleNamespace(points=pack_internal(data)), 'metadata': {'nested': [1]}}
    cache = RouteCache(2, 2**20)
    assert cache.put('first', value)
    assert type(cache.items['first'][0]['result'].points) is InternalPoints
    assert cache.items['first'][0]['result'].points is value['result'].points
    assert cache.bytes == resident_bytes(cache.items['first'][0])
    value['metadata']['nested'].append(2)
    data[0]['x'] = -5
    delivered = cache.get('first')
    assert type(delivered['result'].points) is list
    assert delivered['result'].points == rows() and delivered['metadata']['nested'] == [1]
    delivered['result'].points[0]['x'] = 9
    delivered['metadata']['nested'].append(3)
    assert cache.get('first')['result'].points == rows()
    assert cache.get('first')['metadata']['nested'] == [1]
    cache.remove('first')
    assert cache.bytes == 0


def test_unknown_cache_records_keep_original_recursive_copy(monkeypatch):
    from three_yd_v5.v9_r9_records import copy_for_cache
    from types import SimpleNamespace
    monkeypatch.setenv('THREEYD_V9_R9_MODE', 'P')
    value = {'result': SimpleNamespace(points=[{'nested': {'array': np.arange(3)}}])}
    copied = copy_for_cache(value)
    copied['result'].points[0]['nested']['array'][0] = 8
    assert value['result'].points[0]['nested']['array'][0] == 0
