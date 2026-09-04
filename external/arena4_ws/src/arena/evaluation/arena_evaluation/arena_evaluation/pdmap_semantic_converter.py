"""Convert Pudu ``.pdmap`` semantic fields into ``SemanticMapV1``.

The source archive is read as a ZIP container but is never copied into public
fixtures.  Conversion deliberately does not infer lane direction from polygon
vertex order.  ``right_hand_drive`` is retained as traffic policy metadata;
the actual right side is derived from each selected L1 route at query time.
"""

from __future__ import annotations

import argparse
import io
import json
import math
import zipfile
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml
from PIL import Image

from .semantic_map import (
    ConversionDiagnostic,
    SemanticFeature,
    SemanticMapV1,
    clip_polygon_to_bounds,
    close_polygon,
    polygon_area,
    polygon_self_intersects,
    sha256_file,
)


CLASS_POLICY: Dict[str, Dict[str, Any]] = {
    "forbidden": {"hard": True, "soft": False, "non_stopping": False, "priority": 100},
    "no_go": {"hard": True, "soft": False, "non_stopping": False, "priority": 100},
    "no_stopping": {"hard": False, "soft": False, "non_stopping": True, "priority": 90},
    "junction_area": {"hard": False, "soft": True, "non_stopping": False, "priority": 80},
    "parking_area": {"hard": False, "soft": True, "non_stopping": False, "priority": 70},
    "lane": {"hard": False, "soft": True, "non_stopping": False, "priority": 60},
    "speed_bumps": {"hard": False, "soft": True, "non_stopping": False, "priority": 40},
    # The name alone is not proof that the zone is an impassable physical
    # entity.  r0 preserves it for auditing but does not make it lethal.
    "fence_area": {"hard": False, "soft": False, "non_stopping": False, "priority": 20},
}
TYPE_ALIASES = {
    "forbidden_area": "forbidden",
    "no_go_area": "no_go",
    "prohibition_area": "no_go",
    "no_parking": "no_stopping",
    "forbidden_parking": "no_stopping",
    "speed_bump": "speed_bumps",
}


def _read_archive(path: Path) -> Tuple[Mapping[str, Any], Mapping[str, Any], Image.Image]:
    with zipfile.ZipFile(path) as archive:
        names = set(archive.namelist())
        required = {"ATLAS_DATA", "optemap.yaml", "optemap.pgm"}
        missing = sorted(required - names)
        if missing:
            raise ValueError(f"pdmap missing required members: {missing}")
        # Reject path traversal even though conversion does not extract files.
        unsafe = [name for name in names if Path(name).is_absolute() or ".." in Path(name).parts]
        if unsafe:
            raise ValueError(f"pdmap contains unsafe member names: {unsafe[:3]}")
        atlas = json.loads(archive.read("ATLAS_DATA").decode("utf-8"))
        map_yaml = yaml.safe_load(archive.read("optemap.yaml")) or {}
        image = Image.open(io.BytesIO(archive.read("optemap.pgm"))).convert("L").copy()
    return atlas, map_yaml, image


def _read_extracted(directory: Path) -> Tuple[Mapping[str, Any], Mapping[str, Any], Image.Image]:
    atlas = json.loads((directory / "ATLAS_DATA").read_text(encoding="utf-8"))
    map_yaml = yaml.safe_load((directory / "optemap.yaml").read_text(encoding="utf-8")) or {}
    image = Image.open(directory / "optemap.pgm").convert("L").copy()
    return atlas, map_yaml, image


def _vector_points(value: Any) -> List[List[float]]:
    if not isinstance(value, list) or len(value) % 2:
        return []
    result: List[List[float]] = []
    for index in range(0, len(value), 2):
        try:
            result.append([float(value[index]), float(value[index + 1])])
        except (TypeError, ValueError):
            return []
    return result


def _node_points(value: Any) -> List[List[float]]:
    if not isinstance(value, list):
        return []
    result: List[List[float]] = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            return []
        try:
            result.append([float(point[0]), float(point[1])])
        except (TypeError, ValueError):
            return []
    return result


def _normalize_class(value: Any) -> str:
    raw = str(value or "").strip().lower()
    return TYPE_ALIASES.get(raw, raw)


def _direction_rule(semantic_class: str, source: Mapping[str, Any]) -> str:
    explicit = source.get("direction") or source.get("lane_direction") or source.get("one_way_direction")
    if explicit is not None:
        return "explicit"
    if semantic_class == "lane":
        return "route_tangent_right"
    return "none"


def _feature(
    source: Mapping[str, Any], semantic_class: str, points: Sequence[Sequence[float]],
    source_field: str, bounds: Sequence[float], diagnostics: List[ConversionDiagnostic],
) -> Optional[SemanticFeature]:
    semantic_id = str(source.get("id") or source_field)
    if len(points) < 3 or not all(
        len(point) >= 2 and math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
        for point in points
    ):
        diagnostics.append(ConversionDiagnostic(
            "EMPTY_OR_INVALID_GEOMETRY", "error", "polygon has fewer than three finite points",
            source_field, semantic_id,
        ))
        return None
    polygon = close_polygon(points)
    if len({tuple(point) for point in polygon[:-1]}) < 3:
        diagnostics.append(ConversionDiagnostic(
            "DEGENERATE_POLYGON", "error", "polygon has fewer than three unique points",
            source_field, semantic_id,
        ))
        return None
    if polygon_self_intersects(polygon):
        diagnostics.append(ConversionDiagnostic(
            "SELF_INTERSECTING_POLYGON", "error", "self-intersecting polygon was rejected",
            source_field, semantic_id,
        ))
        return None
    if abs(polygon_area(polygon)) < 1e-8:
        diagnostics.append(ConversionDiagnostic(
            "DEGENERATE_POLYGON", "error", "polygon has zero area",
            source_field, semantic_id,
        ))
        return None
    min_x, min_y, max_x, max_y = (float(value) for value in bounds)
    outside = any(
        point[0] < min_x or point[0] > max_x or point[1] < min_y or point[1] > max_y
        for point in polygon
    )
    if outside:
        polygon = clip_polygon_to_bounds(polygon, bounds)
        diagnostics.append(ConversionDiagnostic(
            "POLYGON_CLIPPED_TO_MAP", "warning", "out-of-bounds polygon was clipped to map extent",
            source_field, semantic_id,
        ))
        if len(polygon) < 4 or abs(polygon_area(polygon)) < 1e-8:
            diagnostics.append(ConversionDiagnostic(
                "POLYGON_OUTSIDE_MAP", "error", "polygon has no valid in-bounds area",
                source_field, semantic_id,
            ))
            return None
    policy = CLASS_POLICY[semantic_class]
    properties: Dict[str, Any] = {
        "source_name": str(source.get("name") or ""),
        "polygon_closed_by_converter": bool(points and list(points[0][:2]) != list(points[-1][:2])),
    }
    explicit = source.get("direction") or source.get("lane_direction") or source.get("one_way_direction")
    if explicit is not None:
        properties["explicit_direction"] = explicit
    if semantic_class == "fence_area":
        properties.update({
            "confirmed_impassable_entity": False,
            "r0_treatment": "diagnostic_only_non_lethal",
        })
        diagnostics.append(ConversionDiagnostic(
            "FENCE_MEANING_UNCONFIRMED", "warning",
            "fence_area has no explicit impassable attribute; it remains non-lethal in r0",
            source_field, semantic_id,
        ))
    if semantic_class == "lane" and explicit is None:
        diagnostics.append(ConversionDiagnostic(
            "LANE_DIRECTION_QUERY_DERIVED", "info",
            "lane has no explicit direction; right side will use the selected L1 route tangent",
            source_field, semantic_id,
        ))
    return SemanticFeature(
        semantic_id=semantic_id,
        semantic_class=semantic_class,
        geometry_type="polygon",
        coordinates=polygon,
        hard=bool(policy["hard"]),
        soft=bool(policy["soft"]),
        non_stopping=bool(policy["non_stopping"]),
        direction_rule=_direction_rule(semantic_class, source),
        priority=int(policy["priority"]),
        source_field=source_field,
        properties=properties,
    )


def convert_payload(
    atlas: Mapping[str, Any], map_yaml: Mapping[str, Any], image_size: Sequence[int],
    *, source_pdmap_hash: str,
) -> SemanticMapV1:
    resolution = float(map_yaml.get("resolution", 0.0))
    origin_value = map_yaml.get("origin") or []
    if len(origin_value) != 3:
        raise ValueError("optemap.yaml origin must contain three values")
    origin = tuple(float(value) for value in origin_value)
    width, height = int(image_size[0]), int(image_size[1])
    if not math.isclose(resolution, 0.05, abs_tol=1e-12):
        raise ValueError(f"PLN-02 requires 0.05 m/cell, got {resolution}")
    epsilon = max(1e-9, resolution * 1e-6)
    bounds = (
        origin[0], origin[1],
        origin[0] + width * resolution - epsilon,
        origin[1] + height * resolution - epsilon,
    )
    diagnostics: List[ConversionDiagnostic] = []
    unrecognized: List[str] = []
    features: List[SemanticFeature] = []
    map_value = atlas.get("map")
    if not isinstance(map_value, Mapping):
        raise ValueError("ATLAS_DATA does not contain a map object")
    for key in map_value:
        if key not in {"zones", "elements"}:
            unrecognized.append(f"map.{key}")
    zones = map_value.get("zones") or []
    if not isinstance(zones, list):
        diagnostics.append(ConversionDiagnostic(
            "INVALID_ZONES_CONTAINER", "error", "map.zones must be a list", "map.zones",
        ))
        zones = []
    for index, zone in enumerate(zones):
        source_field = f"map.zones[{index}]"
        if not isinstance(zone, Mapping):
            unrecognized.append(source_field)
            continue
        semantic_class = _normalize_class(zone.get("type") or zone.get("mode"))
        if semantic_class not in CLASS_POLICY:
            unrecognized.append(f"{source_field}.type={zone.get('type')!r}")
            diagnostics.append(ConversionDiagnostic(
                "UNRECOGNIZED_SEMANTIC_TYPE", "warning",
                f"unrecognized zone type {zone.get('type')!r}", source_field,
                str(zone.get("id") or ""),
            ))
            continue
        points = _node_points(zone.get("nodes")) or _vector_points(zone.get("vector"))
        value = _feature(zone, semantic_class, points, source_field, bounds, diagnostics)
        if value is not None:
            features.append(value)
    traffic_rules: Dict[str, Any] = {}
    elements = map_value.get("elements") or []
    if not isinstance(elements, list):
        diagnostics.append(ConversionDiagnostic(
            "INVALID_ELEMENTS_CONTAINER", "error", "map.elements must be a list", "map.elements",
        ))
        elements = []
    for index, element in enumerate(elements):
        source_field = f"map.elements[{index}]"
        if not isinstance(element, Mapping):
            unrecognized.append(source_field)
            continue
        raw_type = element.get("type") or element.get("mode")
        semantic_class = _normalize_class(raw_type)
        if semantic_class == "traffic_rule":
            traffic_rules.update({
                key: value for key, value in element.items()
                if key not in {"id", "name", "type", "mode", "vector"}
            })
            continue
        if semantic_class in CLASS_POLICY:
            points = _node_points(element.get("nodes")) or _vector_points(element.get("vector"))
            value = _feature(element, semantic_class, points, source_field, bounds, diagnostics)
            if value is not None:
                features.append(value)
        elif raw_type not in {"source", "area", None}:
            unrecognized.append(f"{source_field}.type={raw_type!r}")
    if traffic_rules.get("right_hand_drive") is True:
        diagnostics.append(ConversionDiagnostic(
            "RIGHT_HAND_TRAFFIC_POLICY_ONLY", "info",
            "right_hand_drive records traffic convention; it does not make unmarked lanes one-way",
            "map.elements[type=traffic_rule].right_hand_drive",
        ))
    result = SemanticMapV1(
        frame_id="map", resolution=resolution, origin=origin,
        width=width, height=height, source_pdmap_hash=str(source_pdmap_hash),
        features=features, traffic_rules=traffic_rules,
        diagnostics=diagnostics, unrecognized_fields=unrecognized,
    )
    return result


def _preview(base: Image.Image, semantic_map: SemanticMapV1, path: Path) -> None:
    gray = np.asarray(base.convert("L"), dtype=np.uint8)
    canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    overlay = canvas.copy()
    colors = {
        "forbidden": (20, 20, 230), "no_go": (20, 20, 230),
        "no_stopping": (20, 120, 230), "junction_area": (230, 210, 30),
        "lane": (80, 200, 80), "parking_area": (220, 80, 180),
        "speed_bumps": (20, 180, 230), "fence_area": (180, 100, 30),
    }
    for feature in sorted(semantic_map.features, key=lambda item: item.priority):
        cells = []
        for x, y in feature.coordinates:
            col = math.floor((x - semantic_map.origin[0]) / semantic_map.resolution)
            row_bottom = math.floor((y - semantic_map.origin[1]) / semantic_map.resolution)
            row = semantic_map.height - 1 - row_bottom
            cells.append([int(np.clip(col, 0, semantic_map.width - 1)), int(np.clip(row, 0, semantic_map.height - 1))])
        if len(cells) >= 3:
            cv2.fillPoly(overlay, [np.asarray(cells, dtype=np.int32)], colors[feature.semantic_class])
            cv2.polylines(canvas, [np.asarray(cells, dtype=np.int32)], True, colors[feature.semantic_class], 2)
    canvas = cv2.addWeighted(overlay, 0.35, canvas, 0.65, 0.0)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), canvas):
        raise OSError(f"failed to write semantic preview: {path}")


def convert_pdmap(
    *, pdmap: Optional[str | Path] = None, extracted_dir: Optional[str | Path] = None,
    source_pdmap_hash: str = "", output_dir: Optional[str | Path] = None,
    overwrite: bool = False,
) -> Tuple[SemanticMapV1, Optional[Path]]:
    if (pdmap is None) == (extracted_dir is None):
        raise ValueError("provide exactly one of pdmap or extracted_dir")
    if pdmap is not None:
        source = Path(pdmap).resolve()
        digest = sha256_file(source)
        atlas, map_yaml, image = _read_archive(source)
    else:
        source = Path(extracted_dir).resolve()  # type: ignore[arg-type]
        digest = str(source_pdmap_hash)
        if not digest:
            raise ValueError("source_pdmap_hash is required with extracted_dir")
        atlas, map_yaml, image = _read_extracted(source)
    result = convert_payload(atlas, map_yaml, image.size, source_pdmap_hash=digest)
    if output_dir is None:
        return result, None
    destination = Path(output_dir).resolve()
    if destination.exists() and any(destination.iterdir()) and not overwrite:
        raise FileExistsError(f"refusing to overwrite non-empty output: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    result.save(destination / "semantic_map_v1.json", overwrite=overwrite)
    _preview(image, result, destination / "semantic_overlay.png")
    report = {
        "schema_version": result.schema_version,
        "semantic_map_hash": result.semantic_map_hash,
        "source_pdmap_hash": result.source_pdmap_hash,
        "resolution": result.resolution,
        "origin": list(result.origin),
        "width": result.width,
        "height": result.height,
        "class_counts": result.class_counts(),
        "diagnostic_counts": dict(sorted(Counter(item.code for item in result.diagnostics).items())),
        "unrecognized_fields": result.unrecognized_fields,
        "traffic_rules": result.traffic_rules,
        "source": str(source),
    }
    report_path = destination / "conversion_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result, destination


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert a Pudu pdmap into versioned SemanticMapV1")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pdmap", type=Path, help="source .pdmap ZIP container")
    group.add_argument("--extracted-dir", type=Path, help="ignored directory containing ATLAS_DATA/optemap.*")
    parser.add_argument("--source-pdmap-hash", default="", help="required provenance hash with --extracted-dir")
    parser.add_argument("--output-dir", type=Path, required=True, help="new ignored conversion output directory")
    parser.add_argument("--overwrite", action="store_true", help="overwrite named conversion files (never the pdmap)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    result, destination = convert_pdmap(
        pdmap=args.pdmap, extracted_dir=args.extracted_dir,
        source_pdmap_hash=args.source_pdmap_hash, output_dir=args.output_dir,
        overwrite=bool(args.overwrite),
    )
    print(json.dumps({
        "output_directory": str(destination),
        "semantic_map_hash": result.semantic_map_hash,
        "source_pdmap_hash": result.source_pdmap_hash,
        "class_counts": result.class_counts(),
        "diagnostic_count": len(result.diagnostics),
    }, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
