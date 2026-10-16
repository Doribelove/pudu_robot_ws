"""Map-bound frozen query-set defaults for semantic planning experiments."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from .planner_benchmark.map_utils import sha256_file
from .planner_benchmark.models import Query
from .semantic_map import canonical_hash
from .semantic_query_set import QueryIntent


DEFAULT_QUERY_SET_ID = "pudu_wanda_3f_semantic_compare_selected8_r2_v2"
DEFAULT_QUERY_SET_FILENAME = "pudu_wanda_3f_selected8_gt50m_r2_v2.yaml"
DEFAULT_QUERY_SET_PATH = Path(__file__).resolve().parents[1] / "config" / DEFAULT_QUERY_SET_FILENAME
DEFAULT_MAP_HASH = "05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a"
DEFAULT_SEMANTIC_MAP_HASH = "2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553"
DEFAULT_QUERY_HASH = "7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e"
DEFAULT_QUERY_SET_SHA256 = "b3307d4578447131e71db16156cd2f72e9f5042f98bdce0d75d21fe81300738d"
DEFAULT_QUERY_IDS = (
    "cmp2-01-lane-north",
    "cmp2-02-lane-south",
    "cmp2-03-speed-bump",
    "cmp2-04-multi-junction",
    "cmp2-05-junction-turn",
    "cmp2-06-lane-to-parking",
    "cmp2-07-parking-to-lane",
    "cmp2-08-parking-internal",
)
DEFAULT_MINIMUM_ENDPOINT_CLEARANCE_M = 1.5
DEFAULT_MINIMUM_TOPOLOGY_ROUTE_LENGTH_M = 50.0


class QuerySetContractError(ValueError):
    """Raised when a frozen query-set does not match its declared contract."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise QuerySetContractError(message)


def load_query_set(
    path: str | Path,
    *,
    actual_map_hash: Optional[str] = None,
    actual_semantic_map_hash: Optional[str] = None,
    require_default_contract: bool = False,
) -> Tuple[List[Query], List[QueryIntent], Dict[str, Any]]:
    """Load a frozen query set and fail closed on identity or contract drift."""

    target = Path(path).resolve()
    _require(target.is_file(), f"query-set file does not exist: {target}")
    payload = yaml.safe_load(target.read_text(encoding="utf-8"))
    _require(isinstance(payload, dict), "query-set root must be a mapping")
    raw_queries = payload.get("queries")
    raw_intents = payload.get("intent_validation")
    _require(isinstance(raw_queries, list) and raw_queries, "query-set queries must be a non-empty list")
    _require(isinstance(raw_intents, list), "query-set intent_validation must be a list")

    query_ids = tuple(str(item.get("query_id", "")) for item in raw_queries)
    intent_ids = tuple(str(item.get("query_id", "")) for item in raw_intents)
    _require(len(query_ids) == len(set(query_ids)), "query-set contains duplicate query IDs")
    _require(query_ids == intent_ids, "query and intent IDs/order must match exactly")
    computed_query_hash = canonical_hash(
        [
            {key: item[key] for key in ("query_id", "start", "goal", "category")}
            for item in raw_queries
        ]
    )
    _require(
        payload.get("query_hash") == computed_query_hash,
        "query-set declared query_hash does not match its query content",
    )
    if actual_map_hash is not None:
        _require(
            payload.get("map_hash") == actual_map_hash,
            f"query-set map hash mismatch: expected {actual_map_hash}, got {payload.get('map_hash')}",
        )
    if actual_semantic_map_hash is not None:
        _require(
            payload.get("semantic_map_hash") == actual_semantic_map_hash,
            "query-set semantic-map hash does not match the active semantic map",
        )

    if require_default_contract:
        _require(payload.get("schema_version") == DEFAULT_QUERY_SET_ID, "unexpected default query-set ID")
        _require(sha256_file(target) == DEFAULT_QUERY_SET_SHA256, "default query-set file SHA-256 drifted")
        _require(payload.get("map_hash") == DEFAULT_MAP_HASH, "default query-set map hash drifted")
        _require(
            payload.get("semantic_map_hash") == DEFAULT_SEMANTIC_MAP_HASH,
            "default query-set semantic-map hash drifted",
        )
        _require(computed_query_hash == DEFAULT_QUERY_HASH, "default query hash drifted")
        _require(query_ids == DEFAULT_QUERY_IDS, "default query IDs/order drifted")
        _require(len(raw_queries) == 8, "default query set must contain exactly eight queries")
        _require(payload.get("all_endpoints_footprint_safe") is True, "default endpoints are not footprint-safe")
        _require(payload.get("all_endpoints_connected") is True, "default endpoints are not connected")
        _require(payload.get("all_routes_strictly_gt_50m") is True, "default >50 m gate is not declared")
        for intent in raw_intents:
            verification = intent.get("verification") or {}
            route_length = float(verification.get("topology_route_length_m", 0.0))
            clearance = float(verification.get("minimum_endpoint_clearance_m", 0.0))
            _require(
                route_length > DEFAULT_MINIMUM_TOPOLOGY_ROUTE_LENGTH_M,
                f"{intent.get('query_id')}: topology route must be strictly longer than 50 m",
            )
            _require(
                clearance >= DEFAULT_MINIMUM_ENDPOINT_CLEARANCE_M,
                f"{intent.get('query_id')}: endpoint clearance is below 1.5 m",
            )
            _require(intent.get("footprint_safe") is True, f"{intent.get('query_id')}: footprint gate failed")
            _require(intent.get("purpose_verified") is True, f"{intent.get('query_id')}: purpose gate failed")

    try:
        queries = [Query(**item) for item in raw_queries]
        intents = [QueryIntent(**item) for item in raw_intents]
    except (TypeError, KeyError) as error:
        raise QuerySetContractError(f"invalid query-set schema: {error}") from error
    metadata = {
        key: value
        for key, value in payload.items()
        if key not in {"queries", "intent_validation"}
    }
    metadata.update(
        {
            "query_set_id": payload.get("schema_version"),
            "query_set_source": str(target),
            "query_set_file_sha256": sha256_file(target),
            "query_set_source_mode": (
                "map_bound_default" if require_default_contract else "explicit_override"
            ),
        }
    )
    return queries, intents, metadata


__all__ = [
    "DEFAULT_MAP_HASH",
    "DEFAULT_QUERY_HASH",
    "DEFAULT_QUERY_IDS",
    "DEFAULT_QUERY_SET_ID",
    "DEFAULT_QUERY_SET_PATH",
    "DEFAULT_QUERY_SET_SHA256",
    "DEFAULT_SEMANTIC_MAP_HASH",
    "QuerySetContractError",
    "load_query_set",
]
