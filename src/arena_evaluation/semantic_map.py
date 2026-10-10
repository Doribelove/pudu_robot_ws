from __future__ import annotations
import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple
SEMANTIC_MAP_VERSION = 'SemanticMapV1'
KNOWN_SEMANTIC_CLASSES = {'forbidden', 'no_go', 'no_stopping', 'junction_area', 'lane', 'parking_area', 'speed_bumps', 'fence_area', 'unlabelled'}

def canonical_hash(value: Any) -> str:
    """Return a stable SHA-256 for JSON-compatible policy/provenance data."""
    payload = json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False, default=str).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()

@dataclass(frozen=True)
class ConversionDiagnostic:
    code: str
    severity: str
    message: str
    source_field: str = ''
    semantic_id: str = ''

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> 'ConversionDiagnostic':
        return cls(**{key: value.get(key, '') for key in cls.__dataclass_fields__})

@dataclass
class SemanticFeature:
    semantic_id: str
    semantic_class: str
    geometry_type: str
    coordinates: List[List[float]]
    hard: bool = False
    soft: bool = False
    non_stopping: bool = False
    direction_rule: str = 'none'
    priority: int = 0
    source_field: str = ''
    properties: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.semantic_class not in KNOWN_SEMANTIC_CLASSES:
            raise ValueError(f'unknown semantic class: {self.semantic_class}')
        if self.geometry_type not in {'polygon', 'line', 'point'}:
            raise ValueError(f'unsupported geometry type: {self.geometry_type}')
        self.coordinates = [[float(point[0]), float(point[1])] for point in self.coordinates]

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> 'SemanticFeature':
        return cls(semantic_id=str(value['semantic_id']), semantic_class=str(value['semantic_class']), geometry_type=str(value['geometry_type']), coordinates=[list(point) for point in value.get('coordinates', [])], hard=bool(value.get('hard', False)), soft=bool(value.get('soft', False)), non_stopping=bool(value.get('non_stopping', False)), direction_rule=str(value.get('direction_rule', 'none')), priority=int(value.get('priority', 0)), source_field=str(value.get('source_field', '')), properties=dict(value.get('properties') or {}))

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
            raise ValueError(f'unsupported semantic map version: {self.schema_version}')
        if not math.isfinite(float(self.resolution)) or float(self.resolution) <= 0.0:
            raise ValueError('semantic map resolution must be positive')
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError('semantic map dimensions must be positive')
        if len(self.origin) != 3 or not all((math.isfinite(float(v)) for v in self.origin)):
            raise ValueError('semantic map origin must contain three finite values')
        self.origin = tuple((float(value) for value in self.origin))
        self.resolution = float(self.resolution)
        self.width = int(self.width)
        self.height = int(self.height)
        self.features = [value if isinstance(value, SemanticFeature) else SemanticFeature.from_dict(value) for value in self.features]
        self.diagnostics = [value if isinstance(value, ConversionDiagnostic) else ConversionDiagnostic.from_dict(value) for value in self.diagnostics]
        self.unrecognized_fields = sorted(set((str(value) for value in self.unrecognized_fields)))

    @property
    def world_bounds(self) -> Tuple[float, float, float, float]:
        return (self.origin[0], self.origin[1], self.origin[0] + self.width * self.resolution, self.origin[1] + self.height * self.resolution)

    def to_dict(self, *, include_hash: bool=True) -> Dict[str, Any]:
        value: Dict[str, Any] = {'schema_version': self.schema_version, 'frame_id': self.frame_id, 'resolution': self.resolution, 'origin': list(self.origin), 'width': self.width, 'height': self.height, 'source_pdmap_hash': self.source_pdmap_hash, 'features': [asdict(feature) for feature in self.features], 'traffic_rules': self.traffic_rules, 'diagnostics': [asdict(item) for item in self.diagnostics], 'unrecognized_fields': self.unrecognized_fields}
        if include_hash:
            value['semantic_map_hash'] = canonical_hash(value)
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
        expected = (float(hospital_map.resolution), tuple((float(v) for v in hospital_map.origin)), int(hospital_map.width), int(hospital_map.height))
        actual = (self.resolution, self.origin, self.width, self.height)
        if actual != expected:
            raise ValueError(f'semantic/base map metadata mismatch: {actual!r} != {expected!r}')

    def save(self, path: str | Path, *, overwrite: bool=False) -> Path:
        target = Path(path)
        if target.exists() and (not overwrite):
            raise FileExistsError(f'refusing to overwrite semantic map: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        return target

    @classmethod
    def load(cls, path: str | Path) -> 'SemanticMapV1':
        payload = json.loads(Path(path).read_text(encoding='utf-8'))
        expected_hash = str(payload.pop('semantic_map_hash', ''))
        result = cls(frame_id=str(payload['frame_id']), resolution=float(payload['resolution']), origin=tuple(payload['origin']), width=int(payload['width']), height=int(payload['height']), source_pdmap_hash=str(payload.get('source_pdmap_hash', '')), features=[SemanticFeature.from_dict(value) for value in payload.get('features', [])], traffic_rules=dict(payload.get('traffic_rules') or {}), diagnostics=[ConversionDiagnostic.from_dict(value) for value in payload.get('diagnostics', [])], unrecognized_fields=list(payload.get('unrecognized_fields') or []), schema_version=str(payload.get('schema_version', '')))
        if expected_hash and expected_hash != result.semantic_map_hash:
            raise ValueError('semantic map content hash mismatch')
        return result
