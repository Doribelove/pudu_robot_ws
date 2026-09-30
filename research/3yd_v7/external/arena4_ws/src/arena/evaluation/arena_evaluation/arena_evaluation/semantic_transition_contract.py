"""Pure semantic statistics for the user-approved endpoint contract R2.

Safety, endpoint identity, control replay, and planner promotion are separate
gates. Parking input is the existing RegionalPreferenceBuilderR1/R2/R3 field:
1 - min(region_clearance, map_clearance) / component_maximum_clearance.
It is dimensionless, with zero at maximal clearance; it is not metres.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class TransitionContractR2:
    contract_revision: str = "semantic-endpoint-transition-short-full-6m-r2"
    endpoint_transition_each_m: float = 6.0
    short_path_max_m: float = 12.0
    sample_spacing_m: float = 0.025
    lane_correct_side_min: float = 0.80
    lane_target_error_max_m: float = 0.50
    lane_target_band_ratio_exclusive_min: float = 0.50
    parking_normalized_deviation_max: float = 0.25
    parking_center_band_ratio_exclusive_min: float = 0.50

    def __post_init__(self) -> None:
        values = asdict(self)
        if any(not math.isfinite(float(v)) for k, v in values.items() if k != "contract_revision"):
            raise ValueError("contract values must be finite")
        if (self.endpoint_transition_each_m < 0.0 or self.sample_spacing_m <= 0.0
                or self.short_path_max_m != 2.0 * self.endpoint_transition_each_m):
            raise ValueError("short-path boundary must equal twice the endpoint transition")
        if (self.lane_target_error_max_m < 0.0
                or not 0.0 <= self.lane_correct_side_min <= 1.0
                or not 0.0 <= self.lane_target_band_ratio_exclusive_min < 1.0
                or not 0.0 <= self.parking_normalized_deviation_max <= 1.0
                or not 0.0 <= self.parking_center_band_ratio_exclusive_min < 1.0):
            raise ValueError("contract thresholds are outside their valid range")


def resample_path(points: Sequence[Mapping[str, Any]], contract: TransitionContractR2 | None = None):
    """Reuse the existing fixed arc-length sampler; never resample safety here."""
    from .semantic_transition_online_adapter import _resample_points
    return _resample_points(points, (contract or TransitionContractR2()).sample_spacing_m)


def _vector(value, size: int, name: str, *, boolean: bool = False):
    array = np.asarray(value)
    if array.ndim != 1 or len(array) != size:
        raise ValueError(f"{name} must be a one-dimensional array matching station_m")
    if boolean:
        if not np.all(np.isin(array, (False, True))):
            raise ValueError(f"{name} must contain only booleans")
        return array.astype(bool)
    return array.astype(float)


def _class_result(kind: str, mask, full_mask, error, side, contract: TransitionContractR2) -> dict:
    full_count, count = int(np.count_nonzero(full_mask)), int(np.count_nonzero(mask))
    record = {"class": kind, "full_path_class_sample_count": full_count,
              "sample_count": count, "semantic_gate_passed": None,
              "contract_metric_status": "NOT_APPLICABLE", "failure_reason": ""}
    if full_count == 0:
        return record
    record["semantic_gate_passed"] = False
    if count == 0:
        record.update(contract_metric_status="INVALID_CONTRACT_METRIC",
                      failure_reason="EMPTY_APPLICABLE_REGION")
        return record
    values = error[mask]
    if not np.all(np.isfinite(values)) or np.any(values < 0.0) or (kind == "parking" and np.any(values > 1.0)):
        record.update(contract_metric_status="INVALID_CONTRACT_METRIC",
                      failure_reason="INVALID_APPLICABLE_FIELD_VALUES")
        return record
    p50 = float(np.median(values))
    record["contract_metric_status"] = "VALID_CONTRACT_METRIC"
    if kind == "lane":
        correct = side[mask]
        target = correct & (values <= contract.lane_target_error_max_m)
        side_ratio, band = float(np.mean(correct)), float(np.mean(target))
        passed = (side_ratio >= contract.lane_correct_side_min
                  and band > contract.lane_target_band_ratio_exclusive_min
                  and p50 <= contract.lane_target_error_max_m)
        record.update(correct_side_ratio=side_ratio, target_band_ratio=band,
                      lateral_error_p50_m=p50, correct_sample_count=int(np.count_nonzero(correct)),
                      target_sample_count=int(np.count_nonzero(target)))
    else:
        in_band = values <= contract.parking_normalized_deviation_max
        band = float(np.mean(in_band))
        passed = (p50 <= contract.parking_normalized_deviation_max
                  and band > contract.parking_center_band_ratio_exclusive_min)
        record.update(parking_center_normalized_deviation_p50=p50,
                      parking_center_band_ratio=band, center_band_sample_count=int(np.count_nonzero(in_band)),
                      deviation_unit="dimensionless")
    record["semantic_gate_passed"] = bool(passed)
    record["failure_reason"] = "" if passed else "SEMANTIC_GATE_FAILED"
    return record


def audit_transition_samples(*, path_length_m: float, station_m,
                             lane_mask, lane_error_m, lane_correct_side,
                             parking_mask=None, parking_normalized_deviation=None,
                             contract: TransitionContractR2 | None = None,
                             raw_xy=None, baseline_path_length_m: float | None = None) -> dict:
    """Audit already sampled semantic fields with independent class denominators.

    station_m must come from resample_path: global 0.025 m stations plus the
    final endpoint. Missing fields inside a declared class invalidate that
    class; NaNs outside its mask do not participate. No hard safety gate is
    implied by semantic_gate_passed.
    """
    contract = contract or TransitionContractR2()
    length = float(path_length_m)
    station = np.asarray(station_m, dtype=float)
    if not math.isfinite(length) or length < 0.0:
        raise ValueError("path length must be finite and non-negative")
    if station.ndim != 1 or not len(station) or not np.all(np.isfinite(station)):
        raise ValueError("station_m must be a non-empty finite vector")
    expected = np.unique(np.r_[np.arange(0.0, length, contract.sample_spacing_m), length])
    if (len(station) != len(expected) or not np.allclose(station, expected, rtol=0.0, atol=1e-9)
            or (len(station) > 1 and np.any(np.diff(station) <= 0.0))):
        raise ValueError("station_m must use the fixed global arc-length sampling grid")
    n = len(station)
    lane = _vector(lane_mask, n, "lane_mask", boolean=True)
    lane_error = _vector(lane_error_m, n, "lane_error_m")
    lane_side = _vector(lane_correct_side, n, "lane_correct_side", boolean=True)
    parking = np.zeros(n, dtype=bool) if parking_mask is None else _vector(parking_mask, n, "parking_mask", boolean=True)
    parking_error = (np.full(n, np.nan) if parking_normalized_deviation is None
                     else _vector(parking_normalized_deviation, n, "parking_normalized_deviation"))
    short = length <= contract.short_path_max_m
    left, right = (0.0, length) if short else (contract.endpoint_transition_each_m, length-contract.endpoint_transition_each_m)
    active = (station >= left) & (station <= right) if length > 0.0 else np.zeros(n,dtype=bool)
    full = np.ones(n,dtype=bool) if length > 0.0 else np.zeros(n,dtype=bool)

    def report(mask):
        classes = {"lane":_class_result("lane",mask & lane,lane,lane_error,lane_side,contract),
                   "parking":_class_result("parking",mask & parking,parking,parking_error,None,contract)}
        applicable = [r for r in classes.values() if r["contract_metric_status"] != "NOT_APPLICABLE"]
        valid = bool(applicable and all(r["contract_metric_status"] == "VALID_CONTRACT_METRIC" for r in applicable))
        passed = bool(valid and all(r["semantic_gate_passed"] for r in applicable))
        return {"classes":classes,"semantic_gate_passed":passed,
                "contract_metric_status":"VALID_CONTRACT_METRIC" if valid else "INVALID_CONTRACT_METRIC",
                "failure_reason":"" if passed else ("SEMANTIC_GATE_FAILED" if valid else "EMPTY_OR_INVALID_APPLICABLE_REGION"),
                "sample_count":int(np.count_nonzero(mask)),
                "unlabelled_or_other_class_sample_count":int(np.count_nonzero(mask & ~(lane | parking)))}

    diagnostics = {"path_length_m":length,"baseline_path_length_m":baseline_path_length_m,
                   "path_length_ratio_vs_baseline":None,"zero_translation_step_count":None,
                   "endpoint_distance_m":None,"path_length_ratio_vs_endpoint_distance":None,
                   "diagnostic_only_not_additional_acceptance_gate":True}
    if baseline_path_length_m is not None:
        baseline=float(baseline_path_length_m)
        if not math.isfinite(baseline) or baseline <= 0.0:
            raise ValueError("baseline path length must be finite and positive")
        diagnostics["path_length_ratio_vs_baseline"] = length / baseline
    if raw_xy is not None:
        xy=np.asarray(raw_xy,dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or not len(xy) or not np.all(np.isfinite(xy)):
            raise ValueError("raw_xy must be a non-empty finite Nx2 array")
        steps=np.linalg.norm(np.diff(xy,axis=0),axis=1)
        endpoint_distance=float(np.linalg.norm(xy[-1]-xy[0]))
        diagnostics.update(zero_translation_step_count=int(np.count_nonzero(steps<=1e-12)),
                           endpoint_distance_m=endpoint_distance,
                           path_length_ratio_vs_endpoint_distance=length/endpoint_distance if endpoint_distance>1e-12 else None)
    full_result, active_result = report(full), report(active)
    return {"contract_revision":contract.contract_revision,"contract":asdict(contract),
            "path_length_m":length,"short_path_full_semantics":short,
            "active_interval_m":[left,right],"excluded_start_m":left,"excluded_goal_m":length-right,
            "class_overlap_sample_count":int(np.count_nonzero(lane & parking)),
            "full_path":full_result,"active_window":active_result,
            "semantic_gate_passed":active_result["semantic_gate_passed"],
            "contract_metric_status":active_result["contract_metric_status"],
            "full_path_safety_audit_required":True,"manipulation_risk_diagnostics":diagnostics}
