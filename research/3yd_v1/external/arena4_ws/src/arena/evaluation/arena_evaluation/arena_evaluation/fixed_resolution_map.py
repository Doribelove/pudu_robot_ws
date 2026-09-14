"""Create an exact nearest-neighbour fixed-resolution derivative of a static map."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import math
from pathlib import Path
from typing import Dict

import numpy as np
import yaml
from PIL import Image


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


def prepare_fixed_resolution_map(
    source_map: str | Path,
    resolution: float,
    output_dir: str | Path,
) -> Path:
    """Write a derivative map, requiring an exact integer scale factor."""
    source_yaml = Path(source_map).resolve()
    config = yaml.safe_load(source_yaml.read_text()) or {}
    source_image_path = _resolve_image(source_yaml, str(config["image"])).resolve()
    source_image = np.asarray(Image.open(source_image_path).convert("L"))
    if source_image.ndim != 2:
        raise ValueError("source map image must be grayscale")
    source_resolution = float(config["resolution"])
    target_resolution = float(resolution)
    if target_resolution <= 0.0:
        raise ValueError("target resolution must be positive")
    scale_float = source_resolution / target_resolution
    scale = int(round(scale_float))
    if scale < 1 or not math.isclose(scale_float, scale, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(
            f"resolution conversion must be an integer enlargement: {source_resolution} -> {target_resolution}"
        )
    target = np.repeat(np.repeat(source_image, scale, axis=0), scale, axis=1)
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    image_path = output / "map.pgm"
    Image.fromarray(target).save(image_path, format="PPM")

    origin = list(config["origin"])
    source_extent = (source_image.shape[1] * source_resolution, source_image.shape[0] * source_resolution)
    target_extent = (target.shape[1] * target_resolution, target.shape[0] * target_resolution)
    if not np.allclose(source_extent, target_extent, rtol=0.0, atol=1e-9):
        raise ValueError("derived map does not preserve physical extent")
    map_config = dict(config)
    map_config["image"] = image_path.name
    map_config["resolution"] = target_resolution
    map_config["origin"] = origin
    (output / "map.yaml").write_text(yaml.safe_dump(map_config, sort_keys=False))

    source_counts = _class_counts(source_image, config)
    derived_counts = _class_counts(target, config)
    expected_counts = {key: value * scale * scale for key, value in source_counts.items()}
    metadata = {
        "schema_version": 1,
        "map_id": output.name,
        "source_map_id": source_yaml.parent.parent.name if source_yaml.parent.name == "map" else source_yaml.parent.name,
        "source_map_sha256": sha256_file(source_image_path),
        "source_map_yaml_sha256": sha256_file(source_yaml),
        "derived_map_sha256": sha256_file(image_path),
        "derived_map_yaml_sha256": sha256_file(output / "map.yaml"),
        "source_resolution": source_resolution,
        "target_resolution": target_resolution,
        "source_size": [int(source_image.shape[1]), int(source_image.shape[0])],
        "target_size": [int(target.shape[1]), int(target.shape[0])],
        "physical_extent_m": [float(target_extent[0]), float(target_extent[1])],
        "origin": origin,
        "resampling": "nearest_neighbor_exact_replication",
        "scale_factor": scale,
        "dynamic_obstacles": False,
        **source_counts,
        **{f"derived_{key}": value for key, value in derived_counts.items()},
        "expected_derived_counts": {f"derived_{key}": value for key, value in expected_counts.items()},
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    if derived_counts != expected_counts:
        raise ValueError("derived occupancy class counts are not an exact scale replication")
    (output / "metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare an exact fixed-resolution static occupancy map")
    parser.add_argument("--source-map", required=True, help="source map.yaml")
    parser.add_argument("--resolution", required=True, type=float)
    parser.add_argument("--output-dir", required=True)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = prepare_fixed_resolution_map(args.source_map, args.resolution, args.output_dir)
    except (OSError, KeyError, ValueError, yaml.YAMLError) as exc:
        print(f"prepare_fixed_resolution_map: ERROR: {exc}")
        return 2
    print(f"fixed-resolution map: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
