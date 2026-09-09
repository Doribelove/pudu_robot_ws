"""Frozen 2A-V1-r2 L1 -> ROI/ACK -> L3 -> canonical-audit runtime."""

from __future__ import annotations

import os
from pathlib import Path
import time
from typing import Any, Dict

from arena_evaluation import path_audit, topology
from arena_evaluation import two_layer_v2_semantic_benchmark as semantic_runtime
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.semantic_query_defaults import load_query_set

from three_d_v1_nav2.contracts import (
    EXPECTED_MAP_SHA256, EXPECTED_SEMANTIC_MAP_HASH, QUERY_SET, sha256_file,
    verify_static_inputs,
)
from .contracts import verify_frozen_2a_sources
from .frozen_runtime import candidate, r1, r2


class FrozenTwoAPlannerCore:
    """One persistent Smac session with the byte-verified frozen r2 Python pipeline."""

    def __init__(self, run_dir: Path, output_dir: Path, *, l3_domain_id: int) -> None:
        self.run_dir = run_dir.resolve()
        self.output = output_dir.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.source_hashes = verify_frozen_2a_sources()
        from arena_evaluation.semantic_map import SemanticMapV1
        semantic = SemanticMapV1.load(
            self.run_dir / "derived_map/semantic_conversion/semantic_map_v1.json")
        self.inputs = verify_static_inputs(
            map_pgm=self.run_dir / "derived_map/extracted/optemap.pgm",
            map_yaml=self.run_dir / "derived_map/extracted/optemap.yaml",
            semantic_map_hash=semantic.semantic_map_hash,
        )
        self.map_yaml = self.run_dir / "derived_map/extracted/optemap.yaml"
        self.ctx = semantic_runtime._context(self.map_yaml)
        self.topology_dir = self.run_dir / "derived_map/topology_cache"
        self.topology = topology.load_topology(
            self.topology_dir, self.ctx.hospital_map, runtime.FOOTPRINT,
            padding_m=.05, safety_margin_m=.05, allow_unknown=False,
        )
        queries, _, _ = load_query_set(
            QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
            actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
            require_default_contract=True,
        )
        self.queries = {query.query_id: query for query in queries}
        self.source_files, self.source_hash = r2._source_manifest()
        self.cache = r2.R2RouteMaskCache(
            self.ctx, self.topology, self.source_hash,
            self.output / "offline_mask_cache", endpoint_mode="baseline",
        )
        self.cache_manifest = self.cache.prepare(list(queries))
        self.auditor = path_audit.PathAuditor(
            self.ctx, source_commit="57549bcc64d83f752f6560aeb65e5bd7b22bf67e")
        self.spec = runtime.backend_availability()["hybrid_astar"]
        if not self.spec.available:
            raise RuntimeError(self.spec.reason)
        main_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        l3_domain = str(int(l3_domain_id))
        if l3_domain == main_domain:
            raise RuntimeError("2A L3 domain must differ from main Nav2 domain")
        os.environ["ROS_DOMAIN_ID"] = l3_domain
        self.session = candidate.SmacSession(
            self.ctx, self.output / "l3", map_yaml=self.map_yaml,
            log_tag="nav2_frozen_2a_v1_r2", local_mask_updates=True,
            optimization_profile=r1.OPTIMIZATION_PROFILE,
            smac_parameter_profile=r1.SMAC_PARAMETER_PROFILE,
            optimization_stage=r1.OPTIMIZATION_STAGE,
            enable_mask_reuse_noop=True,
            planner_parameter_overrides={"angle_quantization_bins": 48},
            costmap_ack_timeout_s=3.0,
        )
        self.session.local_map_update_strategy = "roi_ack"
        self.session.full_grid_settle_cycles = 0
        try:
            self.session.start()
        except Exception:
            self.session.close()
            raise
        finally:
            os.environ["ROS_DOMAIN_ID"] = main_domain

    def plan(self, query: Any) -> tuple[Any, Dict[str, Any]]:
        started = time.monotonic_ns()
        reset = self.session.reset_query_state(query.query_id, restore_base_map=False)
        result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
            self.ctx, query, self.topology, self.session, self.spec,
            corridor_padding_m=r1.BASE_CORRIDOR_PADDING_M,
            corridor_semantics=r1.CORRIDOR_SEMANTICS,
            padding_schedule_m=(r1.BASE_CORRIDOR_PADDING_M,),
            validate_each_attempt=True,
            cache_mode=candidate.CACHE_MODE_OPTIMIZED,
            corridor_mask_builder=self.cache.builder,
            route_selector=self.cache.route_selector,
            canonical_path_auditor=self.auditor.audit,
            skip_session_path_mask_validation=True,
        )
        fallback = False
        if not (
            result.planner_success and result.points and result.path_audit is not None
            and result.path_audit.final_valid_success
        ):
            optimized = dict(diagnostics)
            reset = self.session.reset_query_state(query.query_id, restore_base_map=True)
            result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
                self.ctx, query, self.topology, self.session, self.spec,
                corridor_padding_m=r1.BASE_CORRIDOR_PADDING_M,
                corridor_semantics=r1.CORRIDOR_SEMANTICS,
                padding_schedule_m=(r1.BASE_CORRIDOR_PADDING_M,),
                force_full_update=True, validate_each_attempt=True,
                cache_mode=candidate.CACHE_MODE_BASELINE,
                canonical_path_auditor=self.auditor.audit,
                skip_session_path_mask_validation=True,
            )
            fallback = True
            diagnostics = {**dict(diagnostics), "optimized_diagnostics": optimized}
        diagnostics = {
            **dict(diagnostics), "fallback_used": fallback, "reset": reset,
            "request_wall_ms": (time.monotonic_ns() - started) / 1e6,
        }
        if not result.planner_success or not result.points:
            raise RuntimeError(result.failure_code or diagnostics.get("failure_code") or "L3_NO_PATH")
        audit = result.path_audit
        if audit is None or not audit.final_valid_success:
            raise RuntimeError(
                (audit.metrics.get("failure_code") if audit else None) or "CANONICAL_PATH_AUDIT_FAILED")
        if diagnostics.get("costmap_update_acknowledged") is not True:
            raise RuntimeError("ROI_CONTENT_ACK_FAILED")
        if int(diagnostics.get("costmap_ack_mismatch_cells") or 0) != 0:
            raise RuntimeError("ROI_CONTENT_ACK_MISMATCH")
        if int(diagnostics.get("l2_call_count") or 0) != 0:
            raise RuntimeError("2A_ARCHITECTURE_UNEXPECTED_L2_CALL")
        return result, diagnostics

    def close(self) -> None:
        self.session.close()


def topology_hashes(run_dir: Path) -> Dict[str, str]:
    directory = run_dir / "derived_map/topology_cache"
    return {
        name: sha256_file(directory / name)
        for name in ("topology_arrays.npz", "topology_graph.json", "topology_metadata.yaml")
    }
