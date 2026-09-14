"""Versioned, planner-independent semantic-map representation for PLN-02.

``SemanticMapV1`` is the single normalized input shared by the 2A-V2 L1 and
L3 implementations.  Geometry is stored in map-frame world coordinates; PGM
row inversion is deliberately deferred to :mod:`semantic_rasterizer`, where
the authoritative ``HospitalMap.world_to_cell`` conversion is available.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SEMANTIC_MAP_VERSION = "SemanticMapV1"
KNOWN_SEMANTIC_CLASSES = {
    "forbidden", "no_go", "no_stopping", "junction_area", "lane",
    "parking_area", "speed_bumps", "fence_area", "unlabelled",
}

WorldPoint = Tuple[float, float]


def canonical_hash(value: Any) -> str:
    """Return a stable SHA-256 for JSON-compatible policy/provenance data."""
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False, default=str,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class ConversionDiagnostic:
    code: str
    severity: str
    message: str
    source_field: str = ""
    semantic_id: str = ""

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "ConversionDiagnostic":
        return cls(**{key: value.get(key, "") for key in cls.__dataclass_fields__})


@dataclass
class SemanticFeature:
    semantic_id: str
    semantic_class: str
    geometry_type: str
    coordinates: List[List[float]]
    hard: bool = False
    soft: bool = False
    non_stopping: bool = False
    direction_rule: str = "none"
    priority: int = 0
    source_field: str = ""
    properties: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.semantic_class not in KNOWN_SEMANTIC_CLASSES:
            raise ValueError(f"unknown semantic class: {self.semantic_class}")
        if self.geometry_type not in {"polygon", "line", "point"}:
            raise ValueError(f"unsupported geometry type: {self.geometry_type}")
        self.coordinates = [
            [float(point[0]), float(point[1])] for point in self.coordinates
        ]

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "SemanticFeature":
        return cls(
            semantic_id=str(value["semantic_id"]),
            semantic_class=str(value["semantic_class"]),
            geometry_type=str(value["geometry_type"]),
            coordinates=[list(point) for point in value.get("coordinates", [])],
            hard=bool(value.get("hard", False)),
            soft=bool(value.get("soft", False)),
            non_stopping=bool(value.get("non_stopping", False)),
            direction_rule=str(value.get("direction_rule", "none")),
            priority=int(value.get("priority", 0)),
            source_field=str(value.get("source_field", "")),
            properties=dict(value.get("properties") or {}),
        )


@dataclass
class SemanticMapV1:
    frame_id: str
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int
    source_pdmap_hash: str
    features: List[SemanticFeature] = field(default_factory=list)
    traffic_rules: Dict[str, Any] = field(default_factory=dict)
    diagnostics: List[ConversionDiagnostic] = field(default_factory=list)
    unrecognized_fields: List[str] = field(default_factory=list)
    schema_version: str = SEMANTIC_MAP_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != SEMANTIC_MAP_VERSION:
            raise ValueError(f"unsupported semantic map version: {self.schema_version}")
        if not math.isfinite(float(self.resolution)) or float(self.resolution) <= 0.0:
            raise ValueError("semantic map resolution must be positive")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("semantic map dimensions must be positive")
        if len(self.origin) != 3 or not all(math.isfinite(float(v)) for v in self.origin):
            raise ValueError("semantic map origin must contain three finite values")
        self.origin = tuple(float(value) for value in self.origin)
        self.resolution = float(self.resolution)
        self.width = int(self.width)
        self.height = int(self.height)
        self.features = [
            value if isinstance(value, SemanticFeature) else SemanticFeature.from_dict(value)
            for value in self.features
        ]
        self.diagnostics = [
            value if isinstance(value, ConversionDiagnostic) else ConversionDiagnostic.from_dict(value)
            for value in self.diagnostics
        ]
        self.unrecognized_fields = sorted(set(str(value) for value in self.unrecognized_fields))

    @property
    def world_bounds(self) -> Tuple[float, float, float, float]:
        return (
            self.origin[0], self.origin[1],
            self.origin[0] + self.width * self.resolution,
            self.origin[1] + self.height * self.resolution,
        )

    def to_dict(self, *, include_hash: bool = True) -> Dict[str, Any]:
        value: Dict[str, Any] = {
            "schema_version": self.schema_version,
            "frame_id": self.frame_id,
            "resolution": self.resolution,
            "origin": list(self.origin),
            "width": self.width,
            "height": self.height,
            "source_pdmap_hash": self.source_pdmap_hash,
            "features": [asdict(feature) for feature in self.features],
            "traffic_rules": self.traffic_rules,
            "diagnostics": [asdict(item) for item in self.diagnostics],
            "unrecognized_fields": self.unrecognized_fields,
        }
        if include_hash:
            value["semantic_map_hash"] = canonical_hash(value)
        return value

    @property
    def semantic_map_hash(self) -> str:
        return canonical_hash(self.to_dict(include_hash=False))

    def class_counts(self) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for feature in self.features:
            counts[feature.semantic_class] = counts.get(feature.semantic_class, 0) + 1
        return dict(sorted(counts.items()))

    def validate_against_map(self, hospital_map: Any) -> None:
        expected = (
            float(hospital_map.resolution), tuple(float(v) for v in hospital_map.origin),
            int(hospital_map.width), int(hospital_map.height),
        )
        actual = (self.resolution, self.origin, self.width, self.height)
        if actual != expected:
            raise ValueError(f"semantic/base map metadata mismatch: {actual!r} != {expected!r}")

    def save(self, path: str | Path, *, overwrite: bool = False) -> Path:
        target = Path(path)
        if target.exists() and not overwrite:
            raise FileExistsError(f"refusing to overwrite semantic map: {target}")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return target

    @classmethod
    def load(cls, path: str | Path) -> "SemanticMapV1":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        expected_hash = str(payload.pop("semantic_map_hash", ""))
        result = cls(
            frame_id=str(payload["frame_id"]),
            resolution=float(payload["resolution"]),
            origin=tuple(payload["origin"]),
            width=int(payload["width"]),
            height=int(payload["height"]),
            source_pdmap_hash=str(payload.get("source_pdmap_hash", "")),
            features=[SemanticFeature.from_dict(value) for value in payload.get("features", [])],
            traffic_rules=dict(payload.get("traffic_rules") or {}),
            diagnostics=[ConversionDiagnostic.from_dict(value) for value in payload.get("diagnostics", [])],
            unrecognized_fields=list(payload.get("unrecognized_fields") or []),
            schema_version=str(payload.get("schema_version", "")),
        )
        if expected_hash and expected_hash != result.semantic_map_hash:
            raise ValueError("semantic map content hash mismatch")
        return result


def polygon_area(points: Sequence[Sequence[float]]) -> float:
    if len(points) < 3:
        return 0.0
    values = points[:-1] if points[0] == points[-1] else points
    return 0.5 * sum(
        float(first[0]) * float(second[1]) - float(second[0]) * float(first[1])
        for first, second in zip(values, values[1:] + values[:1])
    )


def close_polygon(points: Iterable[Sequence[float]]) -> List[List[float]]:
    result: List[List[float]] = []
    for point in points:
        if len(point) < 2:
            continue
        value = [float(point[0]), float(point[1])]
        if not result or value != result[-1]:
            result.append(value)
    if result and result[0] != result[-1]:
        result.append(list(result[0]))
    return result


def _orientation(a: Sequence[float], b: Sequence[float], c: Sequence[float]) -> float:
    return ((float(b[0]) - float(a[0])) * (float(c[1]) - float(a[1]))
            - (float(b[1]) - float(a[1])) * (float(c[0]) - float(a[0])))


def _on_segment(a: Sequence[float], b: Sequence[float], p: Sequence[float]) -> bool:
    return (
        min(float(a[0]), float(b[0])) - 1e-12 <= float(p[0]) <= max(float(a[0]), float(b[0])) + 1e-12
        and min(float(a[1]), float(b[1])) - 1e-12 <= float(p[1]) <= max(float(a[1]), float(b[1])) + 1e-12
    )


def segments_intersect(
    a: Sequence[float], b: Sequence[float], c: Sequence[float], d: Sequence[float],
) -> bool:
    values = (_orientation(a, b, c), _orientation(a, b, d),
              _orientation(c, d, a), _orientation(c, d, b))
    if values[0] * values[1] < 0.0 and values[2] * values[3] < 0.0:
        return True
    return any(
        abs(value) <= 1e-12 and _on_segment(first, second, point)
        for value, first, second, point in (
            (values[0], a, b, c), (values[1], a, b, d),
            (values[2], c, d, a), (values[3], c, d, b),
        )
    )


def polygon_self_intersects(points: Sequence[Sequence[float]]) -> bool:
    polygon = close_polygon(points)
    segment_count = len(polygon) - 1
    for first in range(segment_count):
        for second in range(first + 1, segment_count):
            if abs(first - second) <= 1 or (first == 0 and second == segment_count - 1):
                continue
            if segments_intersect(
                polygon[first], polygon[first + 1], polygon[second], polygon[second + 1],
            ):
                return True
    return False


def clip_polygon_to_bounds(
    points: Sequence[Sequence[float]], bounds: Sequence[float],
) -> List[List[float]]:
    """Clip a polygon to an axis-aligned world rectangle (Sutherland-Hodgman)."""
    polygon = [list(map(float, point[:2])) for point in close_polygon(points)[:-1]]
    min_x, min_y, max_x, max_y = (float(value) for value in bounds)
    tests = (
        (lambda p: p[0] >= min_x, lambda a, b: [min_x, a[1] + (b[1] - a[1]) * (min_x - a[0]) / ((b[0] - a[0]) or 1e-30)]),
        (lambda p: p[0] <= max_x, lambda a, b: [max_x, a[1] + (b[1] - a[1]) * (max_x - a[0]) / ((b[0] - a[0]) or 1e-30)]),
        (lambda p: p[1] >= min_y, lambda a, b: [a[0] + (b[0] - a[0]) * (min_y - a[1]) / ((b[1] - a[1]) or 1e-30), min_y]),
        (lambda p: p[1] <= max_y, lambda a, b: [a[0] + (b[0] - a[0]) * (max_y - a[1]) / ((b[1] - a[1]) or 1e-30), max_y]),
    )
    for inside, intersection in tests:
        if not polygon:
            break
        output: List[List[float]] = []
        previous = polygon[-1]
        previous_inside = inside(previous)
        for current in polygon:
            current_inside = inside(current)
            if current_inside:
                if not previous_inside:
                    output.append(intersection(previous, current))
                output.append(current)
            elif previous_inside:
                output.append(intersection(previous, current))
            previous, previous_inside = current, current_inside
        polygon = output
    return close_polygon(polygon)


def point_in_polygon(point: Sequence[float], polygon: Sequence[Sequence[float]]) -> bool:
    values = close_polygon(polygon)
    x, y = float(point[0]), float(point[1])
    inside = False
    for first, second in zip(values, values[1:]):
        if abs(_orientation(first, second, (x, y))) <= 1e-12 and _on_segment(first, second, (x, y)):
            return True
        if (first[1] > y) != (second[1] > y):
            x_cross = first[0] + (y - first[1]) * (second[0] - first[0]) / (second[1] - first[1])
            if x < x_cross:
                inside = not inside
    return inside


__all__ = [
    "SEMANTIC_MAP_VERSION", "KNOWN_SEMANTIC_CLASSES", "ConversionDiagnostic",
    "SemanticFeature", "SemanticMapV1", "canonical_hash", "sha256_file",
    "close_polygon", "polygon_area", "polygon_self_intersects",
    "clip_polygon_to_bounds", "point_in_polygon", "segments_intersect",
]
