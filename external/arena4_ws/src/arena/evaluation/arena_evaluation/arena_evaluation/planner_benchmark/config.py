from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import yaml

from .models import Query


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = PACKAGE_ROOT / "config"


def resolve_path(value: str | Path, *, base: Path | None = None) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    candidates = [Path.cwd() / path]
    if base is not None:
        candidates.append(base / path)
    candidates.extend(parent / path for parent in PACKAGE_ROOT.parents)
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return candidates[0].resolve()


def load_yaml(path: str | Path) -> Dict[str, Any]:
    return yaml.safe_load(Path(path).read_text()) or {}


def load_protocol(path: str | Path | None = None) -> Tuple[Path, Dict[str, Any]]:
    protocol_path = resolve_path(path or CONFIG_ROOT / "planner_benchmark_protocol.yaml", base=CONFIG_ROOT)
    return protocol_path, load_yaml(protocol_path)


def load_queries(path: str | Path | None = None) -> Tuple[Path, List[Query]]:
    query_path = resolve_path(path or CONFIG_ROOT / "planner_benchmark_queries_hospital.yaml", base=CONFIG_ROOT)
    content = load_yaml(query_path)
    queries = [
        Query(
            query_id=str(item["query_id"]),
            start=[float(value) for value in item["start"]],
            goal=[float(value) for value in item["goal"]],
            category=str(item.get("category", "unspecified")),
            seed=int(item.get("seed", content.get("seed", 0))),
            validation_status=str(item.get("validation_status", "UNVALIDATED")),
        )
        for item in content.get("queries", [])
    ]
    return query_path, queries


def variant_config_path(config_variant: str, planner: str) -> Path:
    planner_key = "navfn" if planner in {"navfn", "navfn_astar"} else "smac_hybrid"
    return CONFIG_ROOT / f"planner_benchmark_{config_variant}_{planner_key}.yaml"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stack_parameters(
    *,
    protocol: Dict[str, Any],
    planner_config: Dict[str, Any],
) -> Dict[str, Any]:
    footprint = protocol["footprint"]
    footprint_string = "[" + ", ".join("[%.6f, %.6f]" % (float(x), float(y)) for x, y in footprint) + "]"
    variant = protocol.get("variants", {}).get(planner_config.get("config_variant", "product"), {})
    planner_ros = dict(planner_config)
    planner_ros.pop("planner_id", None)
    planner_ros.pop("config_variant", None)
    planner_plugins = planner_ros.pop("planner_plugins", ["GridBased"])
    resolution = float(protocol["resolution"])
    width_cells = int(protocol.get("width_cells", protocol.get("map_width_cells", round(float(protocol.get("width_m", 80.0)) / resolution))))
    height_cells = int(protocol.get("height_cells", protocol.get("map_height_cells", round(float(protocol.get("height_m", 80.0)) / resolution))))
    if width_cells <= 0 or height_cells <= 0:
        raise ValueError("protocol must provide positive width_cells and height_cells")
    origin = protocol.get("origin", [-40.0, -40.0, 0.0])
    return {
        "map_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "yaml_filename": "",
                "topic_name": "/map",
                "frame_id": "map",
            }
        },
        "planner_server": {
            "ros__parameters": {
                "use_sim_time": False,
                "expected_planner_frequency": 20.0,
                "planner_plugins": planner_plugins,
                **planner_ros,
            }
        },
        "planner_server_rclcpp_node": {"ros__parameters": {"use_sim_time": False}},
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "use_sim_time": False,
                    "global_frame": "map",
                    "robot_base_frame": "base_link",
                    "update_frequency": float(protocol.get("costmap_update_frequency", 1.0)),
                    "publish_frequency": float(protocol.get("costmap_publish_frequency", 1.0)),
                    "resolution": resolution,
                    # Nav2 costmap width/height are metres. Keep cell counts
                    # explicit in the protocol so physical extent is stable.
                    "width": int(round(width_cells * resolution)),
                    "height": int(round(height_cells * resolution)),
                    "origin_x": float(origin[0]),
                    "origin_y": float(origin[1]),
                    "rolling_window": False,
                    "track_unknown_space": True,
                    "footprint": footprint_string,
                    "plugins": ["static_layer", "inflation_layer"],
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_topic": "/map",
                        "map_subscribe_transient_local": True,
                        "subscribe_to_updates": bool(protocol.get("static_layer_subscribe_to_updates", False)),
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "inflation_radius": float(variant.get("inflation_radius", 0.25)),
                        "cost_scaling_factor": float(variant.get("cost_scaling_factor", 3.0)),
                    },
                    "always_send_full_costmap": True,
                }
            }
        },
        "global_costmap_client": {"ros__parameters": {"use_sim_time": False}},
        "global_costmap_rclcpp_node": {"ros__parameters": {"use_sim_time": False}},
    }
