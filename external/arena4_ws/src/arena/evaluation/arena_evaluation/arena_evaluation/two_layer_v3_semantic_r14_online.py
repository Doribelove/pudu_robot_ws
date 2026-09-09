"""2A-V3 r14 online engineering preflight and exact-ACK/cache harness.

The targeted preflight deliberately reuses the frozen r13 request/search loop
while replacing only the static prepare and costmap publication boundaries.
This isolates deterministic reinflation and immutable-cache effects before the
expanded-query runner is frozen.  Outputs are labelled preflight evidence and
cannot by themselves promote the architecture.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Any, Sequence

import yaml

from . import semantic_applicability_v3 as applicability
from . import semantic_static_cache_v3 as static_cache
from . import two_layer_v3_semantic_r13_online as r13
from . import two_layer_v3_semantic_r14_benchmark as offline
from .semantic_map import sha256_file
from .semantic_v3_ack_r14 import DeterministicReinflationSessionR14


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r14-parking-dispatch-ack-cache"
PROTOCOL_ID = "PLN-02-2A-V3-R14-PARKING-DISPATCH-ACK-CACHE-ONLINE-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R14-ONLINE-PREFLIGHT-RESULT-V1"
DEFAULT_CONFIG = offline.PACKAGE_ROOT/"config/two_layer_v3_semantic_r14_online.yaml"


def _identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _load(path: Path) -> tuple[dict[str, Any], Path, dict[str, Any], dict[str, Any], Path, dict[str, Any], Path]:
    path = Path(path).resolve()
    current = yaml.safe_load(path.read_text()) or {}
    for key, value in _identity().items():
        if current.get(key) != value:
            raise ValueError(f"r14 online identity mismatch for {key}")
    algorithm_path = Path(str(current["algorithm_config"]["path"]))
    if not algorithm_path.is_absolute():
        algorithm_path = path.parent/algorithm_path
    algorithm_path = algorithm_path.resolve()
    algorithm, parent_algorithm, parent_v2, selected8 = offline._load_config(algorithm_path)
    expected = str(current["algorithm_config"]["sha256"])
    if expected != "PENDING_FREEZE_AFTER_CALIBRATION" and sha256_file(algorithm_path) != expected:
        raise ValueError("r14 online algorithm hash mismatch")
    parent_path = Path(str(current["parent_online"]["path"]))
    if not parent_path.is_absolute():
        parent_path = path.parent/parent_path
    parent_path = parent_path.resolve()
    if sha256_file(parent_path) != str(current["parent_online"]["sha256"]):
        raise ValueError("r14 parent online hash mismatch")
    (
        parent_online, _r13_algorithm_path, _r13_algorithm, _r13_parent,
        _r13_selected8, single_algorithm, single_algorithm_path,
    ) = r13._load_config(parent_path)
    compatible = copy.deepcopy(parent_online)
    compatible.update(_identity())
    compatible["schema_version"] = current["schema_version"]
    compatible["online_interface"].update(current["online_interface"])
    compatible["measurement"] = dict(current["measurement"])
    compatible["engineering_preflight_only"] = True
    compatible["r14_algorithm_configuration"] = algorithm
    return (
        compatible, algorithm_path, parent_algorithm, parent_v2, selected8,
        single_algorithm, single_algorithm_path,
    )


def run(
    *, output: Path, scope: str = "targeted", config_path: Path = DEFAULT_CONFIG,
    ros_domain_id: int | None = None, warmups: int = 0, repetitions: int = 1,
) -> dict[str, Any]:
    if scope != "targeted":
        raise ValueError("r14 boundary-isolation harness is targeted-only")
    loaded = _load(config_path)
    cache_root = Path(
        loaded[0]["r14_algorithm_configuration"]["static_cache"]["root"]
    )
    original = {
        "identity": r13._identity,
        "loader": r13._load_config,
        "session": r13.RoutePhaseV3PlannerSession,
        "prepare": r13.r1._prepare,
        "schema": r13.SCHEMA_VERSION,
    }

    def cached_prepare(extracted, semantic_map, topology, config, *, output=None):
        return static_cache.prepare_static_cached(
            extracted, semantic_map, topology, config, output=output,
            cache_root=cache_root,
            maximum_disk_entries=int(
                loaded[0]["r14_algorithm_configuration"]["static_cache"]["maximum_disk_entries"]
            ),
        )

    try:
        r13._identity = _identity
        r13._load_config = lambda _path: loaded
        r13.RoutePhaseV3PlannerSession = DeterministicReinflationSessionR14
        r13.r1._prepare = cached_prepare
        r13.SCHEMA_VERSION = SCHEMA_VERSION
        result = r13.run(
            output=Path(output), scope=scope, config_path=Path(config_path),
            ros_domain_id=ros_domain_id, warmups=warmups, repetitions=repetitions,
        )
    finally:
        r13._identity = original["identity"]
        r13._load_config = original["loader"]
        r13.RoutePhaseV3PlannerSession = original["session"]
        r13.r1._prepare = original["prepare"]
        r13.SCHEMA_VERSION = original["schema"]
    root = Path(output).resolve()
    applicability._write_json(root/"static_cache_summary.json", static_cache.LAST_CACHE_TELEMETRY)
    applicability._write_json(root/"r14_preflight_scope.json", {
        **_identity(),
        "qualification_scope": "BOUNDARY_ISOLATION_PREFLIGHT_NOT_FORMAL_PROMOTION",
        "tests": ["deterministic_reinflation_exact_ack", "static_cache_restore", "targeted_no_regression"],
    })
    (root/"reproduction_command.txt").write_text(
        f"/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r14_online "
        f"--output {root} --scope targeted --warmups {warmups} --repetitions {repetitions}\n"
    )
    # The frozen r13 runner already owns artifact_hashes.json.  Keep the r14
    # wrapper's post-run additions in a separate manifest so the create-only
    # artifact policy remains fail closed instead of overwriting provenance.
    applicability._write_json(
        root/"r14_wrapper_artifact_hashes.json", applicability._manifest_files(root)
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scope", choices=("targeted",), default="targeted")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--warmups", type=int, default=0)
    parser.add_argument("--repetitions", type=int, default=1)
    args = parser.parse_args(argv)
    result = run(
        output=args.output, scope=args.scope, config_path=args.config,
        ros_domain_id=args.ros_domain_id, warmups=args.warmups,
        repetitions=args.repetitions,
    )
    print(json.dumps({
        "gate_results": result["gate_results"],
        "static_prepare_ms": result["static_prepare_ms"],
        "peak_rss_bytes": result["peak_rss_bytes"],
    }, indent=2, sort_keys=True))
    return 0 if result["gate_results"]["scope_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
