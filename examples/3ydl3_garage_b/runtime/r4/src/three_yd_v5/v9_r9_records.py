"""Immutable internal point columns; deepcopy returns a fresh ordinary list.

Only independently owned flat primitive dictionaries are represented here.
The original planner/audits and mutable delivery format are unchanged. Original
resident_bytes sees the actual compact fields, rather than walking thousands
of dictionaries. Unknown schemas and shared row aliases use the parent path.
"""
from dataclasses import dataclass
from copy import deepcopy
from .v9_records import Points
from .v9_maps import immutable
from .v9_r9_options import enabled


@dataclass(frozen=True)
class InternalPoints:
    packed: Points

    def __len__(self):
        return self.packed.count

    def __iter__(self):
        return iter(self.packed.decode())

    def __getitem__(self, index):
        return self.packed.decode()[index]

    def __deepcopy__(self, memo):
        decoded = self.packed.decode()
        memo[id(self)] = decoded
        return decoded


def pack_internal(points):
    if not enabled() or type(points) is not list or not points:
        return points
    if (any(type(row) is not dict for row in points)
            or len({id(row) for row in points}) != len(points)
            or any(type(key) is not str for row in points for key in row)):
        return points
    packed = Points.encode(points)
    return points if packed is None else InternalPoints(packed)


def copy_for_cache(record):
    """Detach mutable metadata but preserve only the proven immutable columns."""
    result = record.get('result') if type(record) is dict else None
    points = getattr(result, 'points', None)
    if enabled() and type(points) is InternalPoints:
        return deepcopy(record, {id(points): points})
    return deepcopy(record)


def storage_evidence(record):
    result = record.get('result') if isinstance(record, dict) else None
    points = getattr(result, 'points', None)
    packed = type(points) is InternalPoints
    return {'mode_enabled': enabled(), 'internal_points_packed': packed,
            'point_count': len(points) if points is not None else 0,
            'float_buffers_immutable': all(immutable(value) for kind, value in points.packed.columns if kind == 'float') if packed else None,
            'deepcopy_delivery_format': 'ordinary independent mutable list' if packed else 'original',
            'audit_or_ACK_results_cached': False,
            'inspection_outside_request_timing': True}
