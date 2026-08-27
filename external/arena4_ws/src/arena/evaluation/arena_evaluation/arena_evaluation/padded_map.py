"""Create fixed-resolution maps with an open free-space boundary around a map.

The source image is copied byte-for-byte into the centered interior of the
larger image.  The source world coordinates therefore remain unchanged; only
the map origin moves outward by the symmetric padding distance.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image, ImageDraw


# Gate centres are source-image pixel coordinates (column/row).  They are
# deliberately defined in the 80 m source frame, so every padded map uses the
# same physical boundary condition and does not depend on its outer size.
HOSPITAL_GATE_PLAN = (
    {"gate_id": "g00", "side": "top", "center_xy": (708, 382)},
    {"gate_id": "g01", "side": "top", "center_xy": (890, 382)},
    {"gate_id": "g02", "side": "bottom", "center_xy": (620, 1497)},
    {"gate_id": "g03", "side": "bottom", "center_xy": (800, 1497)},
    {"gate_id": "g04", "side": "bottom", "center_xy": (980, 1497)},
    {"gate_id": "g05", "side": "left", "center_xy": (552, 510)},
    {"gate_id": "g06", "side": "left", "center_xy": (552, 1100)},
    {"gate_id": "g07", "side": "right", "center_xy": (1047, 510)},
    {"gate_id": "g08", "side": "right", "center_xy": (1047, 1100)},
    {"gate_id": "g09", "side": "right", "center_xy": (1047, 1400)},
)
GATE_PLAN_VERSION = "hospital_boundary_gates_v1"


def _free_mask(image: np.ndarray, config: Dict[str, object]) -> np.ndarray:
    probability = image.astype(np.float32) / 255.0
    if not bool(config.get("negate", 0)):
        probability = 1.0 - probability
    occupied = probability > float(config.get("occupied_thresh", 0.65))
    return (probability < float(config.get("free_thresh", 0.196))) & ~occupied


def _build_gate_mask(
    source_image: np.ndarray,
    config: Dict[str, object],
    resolution: float,
    width_m: float,
    plan: Sequence[Dict[str, object]],
) -> Tuple[np.ndarray, List[Dict[str, object]], List[Tuple[int, int]]]:
    """Open fixed-width corridors from the source free component to its edge.

    The inner edge is derived from the source free-space bounding box.  For
    each gate we therefore only alter the outer shell between the image edge
    and that edge; pixels in the interior map are left untouched.
    """
    if width_m <= 0.0:
        raise ValueError("gate width must be positive")
    width_cells = int(round(width_m / resolution))
    if width_cells < 2:
        raise ValueError("gate width must cover at least two cells")
    if width_cells % 2:
        width_cells += 1
    free = _free_mask(source_image, config)
    try:
        import cv2  # type: ignore

        _, components = cv2.connectedComponents(free.astype(np.uint8), connectivity=8)
        largest = 1 + int(np.argmax(np.bincount(components.ravel())[1:]))
        body = components == largest
    except Exception:
        body = free
    rows, cols = np.where(body)
    if rows.size == 0:
        raise ValueError("source map has no free component for gate attachment")
    top_edge, bottom_edge = int(rows.min()), int(rows.max())
    left_edge, right_edge = int(cols.min()), int(cols.max())
    opened = np.zeros(source_image.shape, dtype=bool)
    records: List[Dict[str, object]] = []
    half = width_cells // 2
    for spec in plan:
        gate_id = str(spec["gate_id"])
        side = str(spec["side"])
        cx, cy = (int(value) for value in spec["center_xy"])
        if side in ("top", "bottom"):
            if not (0 <= cy < source_image.shape[0]) or not (0 <= cx < source_image.shape[1]):
                raise ValueError(f"{gate_id}: centre is outside source image")
            c0, c1 = cx - half, cx + half
            if c0 < 0 or c1 > source_image.shape[1]:
                raise ValueError(f"{gate_id}: gate width exceeds source image")
            inner = top_edge if side == "top" else bottom_edge
            span = range(0, inner + 1) if side == "top" else range(inner, source_image.shape[0])
            cells = [(row, col) for row in span for col in range(c0, c1)]
        elif side in ("left", "right"):
            if not (0 <= cy < source_image.shape[0]) or not (0 <= cx < source_image.shape[1]):
                raise ValueError(f"{gate_id}: centre is outside source image")
            r0, r1 = cy - half, cy + half
            if r0 < 0 or r1 > source_image.shape[0]:
                raise ValueError(f"{gate_id}: gate width exceeds source image")
            inner = left_edge if side == "left" else right_edge
            span = range(0, inner + 1) if side == "left" else range(inner, source_image.shape[1])
            cells = [(row, col) for row in range(r0, r1) for col in span]
        else:
            raise ValueError(f"{gate_id}: unsupported side {side!r}")
        # The first free row/column at the inner edge must be free across the
        # entire opening.  This prevents a gate from cutting into a room wall.
        if not all(bool(free[row, col]) for row, col in cells if (row == inner if side in ("top", "bottom") else col == inner)):
            raise ValueError(f"{gate_id}: gate inner edge is not uniformly free")
        gate_mask = np.zeros(source_image.shape, dtype=bool)
        for row, col in cells:
            opened[row, col] = True
            gate_mask[row, col] = True
        changed_values = source_image[gate_mask]
        gate_rows, gate_cols = np.where(gate_mask)
        records.append({
            "gate_id": gate_id,
            "side": side,
            "center_source_pixel_xy": [cx, cy],
            "width_m": float(width_cells * resolution),
            "width_cells": width_cells,
            "source_open_bbox_xy": [
                int(gate_cols.min()), int(gate_rows.min()),
                int(gate_cols.max()) + 1, int(gate_rows.max()) + 1,
            ],
            "occupied_cells_opened": int(np.count_nonzero(changed_values == 0)),
            "unknown_cells_opened": int(np.count_nonzero(changed_values == 205)),
        })
    return opened, records, [(int(row), int(col)) for row, col in zip(*np.where(opened))]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve_image(map_yaml: Path, image_value: str) -> Path:
    image = Path(image_value)
    return image if image.is_absolute() else map_yaml.parent / image


def _class_counts(image: np.ndarray, config: Dict[str, object]) -> Dict[str, int]:
    probability = image.astype(np.float32) / 255.0
    if not bool(config.get("negate", 0)):
        probability = 1.0 - probability
    occupied = probability > float(config.get("occupied_thresh", 0.65))
    free = probability < float(config.get("free_thresh", 0.196))
    return {
        "occupied_cell_count": int(np.count_nonzero(occupied)),
        "free_cell_count": int(np.count_nonzero(free)),
        "unknown_cell_count": int(image.size - np.count_nonzero(occupied | free)),
    }


def _preview(
    image: np.ndarray,
    source_box: Tuple[int, int, int, int],
    path: Path,
    gate_boxes: Sequence[Tuple[int, int, int, int]] = (),
) -> None:
    """Write a compact diagnostic PNG with source and gate overlays."""
    preview = Image.fromarray(image, mode="L").convert("RGB")
    draw = ImageDraw.Draw(preview)
    left, top, right, bottom = source_box
    draw.rectangle((left, top, right - 1, bottom - 1), outline=(220, 30, 30), width=max(2, image.shape[0] // 1200))
    for box in gate_boxes:
        gate_left, gate_top, gate_right, gate_bottom = box
        draw.rectangle((gate_left, gate_top, gate_right - 1, gate_bottom - 1), outline=(20, 180, 40), width=max(2, image.shape[0] // 1600))
    # Pillow 9+ exposes ``Image.Resampling``; the system runtime may still
    # provide the older module-level constant.
    nearest = getattr(getattr(Image, "Resampling", Image), "NEAREST")
    preview.thumbnail((1800, 1800), nearest)
    preview.save(path, format="PNG", optimize=True)


def prepare_padded_map(
    source_map: str | Path,
    target_width_m: float,
    target_height_m: float | None,
    output_dir: str | Path,
    *,
    target_resolution: float | None = None,
    boundary_value: int = 254,
    connect_gates: bool = True,
    gate_width_m: float = 1.0,
    gate_plan: Sequence[Dict[str, object]] = HOSPITAL_GATE_PLAN,
) -> Path:
    source_yaml = Path(source_map).resolve()
    config = yaml.safe_load(source_yaml.read_text()) or {}
    source_image_path = _resolve_image(source_yaml, str(config["image"])).resolve()
    source_image = np.asarray(Image.open(source_image_path).convert("L"))
    if source_image.ndim != 2:
        raise ValueError("source map image must be grayscale")
    source_resolution = float(config["resolution"])
    resolution = source_resolution if target_resolution is None else float(target_resolution)
    if resolution <= 0.0 or not math.isclose(resolution, source_resolution, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("padded maps must keep the source resolution exactly")
    if not 0 <= int(boundary_value) <= 255:
        raise ValueError("boundary_value must be an 8-bit grayscale value")
    target_height_m = target_width_m if target_height_m is None else target_height_m
    target_width = int(round(float(target_width_m) / resolution))
    target_height = int(round(float(target_height_m) / resolution))
    source_height, source_width = source_image.shape
    if target_width < source_width or target_height < source_height:
        raise ValueError("target dimensions must be at least as large as the source map")
    pad_x = target_width - source_width
    pad_y = target_height - source_height
    if pad_x % 2 or pad_y % 2:
        raise ValueError("target dimensions must leave symmetric integer-cell padding")
    offset_x, offset_y = pad_x // 2, pad_y // 2
    gate_mask = np.zeros(source_image.shape, dtype=bool)
    gate_records: List[Dict[str, object]] = []
    if connect_gates:
        gate_mask, gate_records, _ = _build_gate_mask(
            source_image, config, resolution, gate_width_m, gate_plan
        )
    source_derived = source_image.copy()
    source_derived[gate_mask] = int(boundary_value)
    target = np.full((target_height, target_width), int(boundary_value), dtype=np.uint8)
    target[offset_y:offset_y + source_height, offset_x:offset_x + source_width] = source_derived

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    image_path = output / "map.pgm"
    Image.fromarray(target).save(image_path, format="PPM")

    source_origin = [float(value) for value in config["origin"]]
    target_origin = [
        source_origin[0] - offset_x * resolution,
        source_origin[1] - offset_y * resolution,
        source_origin[2],
    ]
    target_config = dict(config)
    target_config["image"] = image_path.name
    target_config["resolution"] = resolution
    target_config["origin"] = target_origin
    (output / "map.yaml").write_text(yaml.safe_dump(target_config, sort_keys=False))

    source_counts = _class_counts(source_image, config)
    derived_counts = _class_counts(target, config)
    source_box = (offset_x, offset_y, offset_x + source_width, offset_y + source_height)
    source_extent = [source_width * resolution, source_height * resolution]
    target_extent = [target_width * resolution, target_height * resolution]
    changed_values = source_image[gate_mask]
    source_unchanged_outside_gates = bool(np.array_equal(source_derived[~gate_mask], source_image[~gate_mask]))
    gate_boxes: List[Tuple[int, int, int, int]] = []
    source_origin_xy = (source_origin[0], source_origin[1])
    for gate in gate_records:
        x0, y0, x1, y1 = (int(value) for value in gate["source_open_bbox_xy"])
        gate_boxes.append((offset_x + x0, offset_y + y0, offset_x + x1, offset_y + y1))
        cx, cy = (int(value) for value in gate["center_source_pixel_xy"])
        gate["center_world_xy"] = [
            float(source_origin_xy[0] + (cx + 0.5) * resolution),
            float(source_origin_xy[1] + (source_height - cy - 0.5) * resolution),
        ]
        gate["target_open_bbox_xy"] = [offset_x + x0, offset_y + y0, offset_x + x1, offset_y + y1]
    metadata = {
        "schema_version": 1,
        "map_id": output.name,
        "map_family": "hospital_boundary_padded",
        "boundary_mode": "centered_source_plus_open_free_space_with_fixed_gates",
        "source_block_preserved_exactly": not connect_gates,
        "source_unchanged_outside_gates": source_unchanged_outside_gates,
        "gate_plan_version": GATE_PLAN_VERSION if connect_gates else None,
        "gate_count": len(gate_records),
        "gate_width_m": float(gate_width_m) if connect_gates else None,
        "gates": gate_records,
        "source_occupied_cells_opened": int(np.count_nonzero(changed_values == 0)),
        "source_unknown_cells_opened": int(np.count_nonzero(changed_values == 205)),
        "outer_free_region_connected": True,
        "outer_free_region_connected_to_source_query_space": bool(connect_gates),
        "connectivity_note": (
            "Fixed one-metre boundary gates connect the padded free area to "
            "the source free component; source pixels outside gate corridors "
            "are preserved exactly."
            if connect_gates else
            "The outer free padding is connected internally, but is not "
            "connected to the enclosed source query component without a gate."
        ),
        "source_map_id": source_yaml.parent.parent.name if source_yaml.parent.name == "map" else source_yaml.parent.name,
        "source_map_yaml": str(source_yaml),
        "source_map_yaml_sha256": sha256_file(source_yaml),
        "source_map_sha256": sha256_file(source_image_path),
        "derived_map_sha256": sha256_file(image_path),
        "derived_map_yaml_sha256": sha256_file(output / "map.yaml"),
        "source_resolution": source_resolution,
        "target_resolution": resolution,
        "source_size": [int(source_width), int(source_height)],
        "target_size": [int(target_width), int(target_height)],
        "source_physical_extent_m": source_extent,
        "target_physical_extent_m": target_extent,
        "source_origin": source_origin,
        "target_origin": target_origin,
        "source_pixel_offset_xy": [int(offset_x), int(offset_y)],
        "padding_m_each_side": [float(offset_x * resolution), float(offset_y * resolution)],
        "source_world_bounds": [source_origin[0], source_origin[1], source_origin[0] + source_extent[0], source_origin[1] + source_extent[1]],
        "target_world_bounds": [target_origin[0], target_origin[1], target_origin[0] + target_extent[0], target_origin[1] + target_extent[1]],
        "boundary_value": int(boundary_value),
        "boundary_semantics": "free",
        "copy_rule": "source image copied into centered target block; fixed gate corridors are opened when enabled",
        "dynamic_obstacles": False,
        "preview_file": "map_preview.png",
        **source_counts,
        **{f"derived_{key}": value for key, value in derived_counts.items()},
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    (output / "metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    validation = {
        "schema_version": 1,
        "source_block_exact_match": bool(np.array_equal(source_derived, source_image)),
        "source_unchanged_outside_gates": source_unchanged_outside_gates,
        "gate_plan_version": GATE_PLAN_VERSION if connect_gates else None,
        "gate_count": len(gate_records),
        "gate_width_m": float(gate_width_m) if connect_gates else None,
        "gates": gate_records,
        "source_occupied_cells_opened": int(np.count_nonzero(changed_values == 0)),
        "source_unknown_cells_opened": int(np.count_nonzero(changed_values == 205)),
        "source_world_bounds_preserved": True,
        "target_world_bounds": metadata["target_world_bounds"],
        "unique_source_gray_values": sorted(int(value) for value in np.unique(source_image)),
        "unique_derived_gray_values": sorted(int(value) for value in np.unique(target)),
        "boundary_value": int(boundary_value),
        "dynamic_obstacles": False,
        "source_pixel_offset_xy": [int(offset_x), int(offset_y)],
        "outer_free_region_connected": True,
        "outer_free_region_connected_to_source_query_space": bool(connect_gates),
        "connectivity_note": (
            "Fixed gate corridors are the only source-block changes; all "
            "other source cells remain unchanged."
            if connect_gates else
            "Source block is byte-for-byte preserved; no gate is carved."
        ),
    }
    (output / "validation.yaml").write_text(yaml.safe_dump(validation, sort_keys=False))
    _preview(target, source_box, output / "map_preview.png", gate_boxes)
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create a larger static map with the source map and world coordinates preserved")
    parser.add_argument("--source-map", required=True)
    parser.add_argument("--width-m", required=True, type=float)
    parser.add_argument("--height-m", type=float, default=None)
    parser.add_argument("--resolution", type=float, default=None)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--gate-width-m", type=float, default=1.0)
    parser.add_argument("--no-connect-gates", action="store_true")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = prepare_padded_map(
            args.source_map,
            args.width_m,
            args.height_m,
            args.output_dir,
            target_resolution=args.resolution,
            connect_gates=not args.no_connect_gates,
            gate_width_m=args.gate_width_m,
        )
    except (OSError, KeyError, ValueError, yaml.YAMLError) as exc:
        print(f"prepare_padded_map: ERROR: {exc}")
        return 2
    print(f"padded map: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
