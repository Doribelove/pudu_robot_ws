"""Deterministic fail-closed command guard after the Nav2 TEB controller."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, FrozenSet


@dataclass(frozen=True)
class GuardLimits:
    max_linear_speed: float = 2.50
    max_angular_speed: float = 1.50
    max_linear_accel: float = 2.50
    max_angular_accel: float = 2.00
    maximum_curvature: float = 2.50
    near_zero_linear_speed: float = 1.0e-3
    angular_epsilon: float = 1.0e-4

    def validate(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class GuardSample:
    raw_v: float
    raw_w: float
    out_v: float
    out_w: float
    dt: float
    output_linear_accel: float
    output_angular_accel: float
    curvature: float
    events: FrozenSet[str] = field(default_factory=frozenset)


class CommandGuard:
    """Enforce the artificial forward-only Jackal contract on actual commands."""

    def __init__(self, limits: GuardLimits = GuardLimits()) -> None:
        limits.validate()
        self.limits = limits
        self.previous_v = 0.0
        self.previous_w = 0.0
        self.raw_reverse_distance = 0.0
        self.output_reverse_distance = 0.0
        self.counts: Dict[str, int] = {}

    def reset(self) -> None:
        self.previous_v = 0.0
        self.previous_w = 0.0

    def _count(self, events: set[str]) -> None:
        for event in events:
            self.counts[event] = self.counts.get(event, 0) + 1

    def apply(self, raw_v: float, raw_w: float, dt: float,
              measured_angular_speed: float = 0.0) -> GuardSample:
        events: set[str] = set()
        if not all(math.isfinite(value) for value in (raw_v, raw_w, dt)) or dt <= 0.0:
            events.add("INVALID_INPUT_HARD_STOP")
            raw_v = raw_v if math.isfinite(raw_v) else 0.0
            raw_w = raw_w if math.isfinite(raw_w) else 0.0
            dt = dt if math.isfinite(dt) and dt > 0.0 else 1.0e-3
            return self._safe_stop(raw_v, raw_w, dt, events)

        if raw_v < -1.0e-9:
            events.add("REVERSE_HARD_STOP")
            self.raw_reverse_distance += -raw_v * dt
            return self._safe_stop(raw_v, raw_w, dt, events)
        if abs(raw_v) < self.limits.near_zero_linear_speed and abs(raw_w) > self.limits.angular_epsilon:
            events.add("ROTATE_IN_PLACE_HARD_STOP")
            return self._safe_stop(raw_v, raw_w, dt, events)

        target_v = min(max(raw_v, 0.0), self.limits.max_linear_speed)
        target_w = min(max(raw_w, -self.limits.max_angular_speed), self.limits.max_angular_speed)
        if target_v != raw_v:
            events.add("LINEAR_SPEED_SATURATION")
        if target_w != raw_w:
            events.add("ANGULAR_SPEED_SATURATION")

        # The simulated body's angular motion can settle after its translation.
        # During a stop, retain a few mm/s of forward motion until measured yaw
        # motion has settled; this avoids a physical pivot despite a zero twist.
        if (target_v < 0.005 and abs(target_w) <= self.limits.angular_epsilon and
                self.previous_v >= self.limits.near_zero_linear_speed and
                math.isfinite(measured_angular_speed) and
                abs(measured_angular_speed) > self.limits.angular_epsilon * 0.5):
            target_v = min(self.previous_v, max(
                0.005, 1.1 * abs(measured_angular_speed) / self.limits.maximum_curvature))
            events.add("ANGULAR_SETTLE_FORWARD_HOLD")

        return self._bounded_command(raw_v, raw_w, target_v, target_w, dt, events)

    def _bounded_command(self, raw_v, raw_w, target_v, target_w, dt, events):
        max_dv = self.limits.max_linear_accel * dt
        max_dw = self.limits.max_angular_accel * dt
        low_w = max(-self.limits.max_angular_speed, self.previous_w - max_dw)
        high_w = min(self.limits.max_angular_speed, self.previous_w + max_dw)
        nearest_zero_w = min(max(0.0, low_w), high_w)
        # Deceleration must keep enough forward speed for the angular slew
        # bound. Clipping curvature after independent slew limits can violate
        # the angular acceleration contract.
        minimum_v = abs(nearest_zero_w) / self.limits.maximum_curvature
        if abs(nearest_zero_w) > self.limits.angular_epsilon:
            minimum_v = max(minimum_v, self.limits.near_zero_linear_speed)
        low_v = max(0.0, self.previous_v - max_dv, minimum_v)
        high_v = min(self.limits.max_linear_speed, self.previous_v + max_dv)
        out_v = min(max(target_v, low_v), high_v)
        curvature_limit = self.limits.maximum_curvature * out_v
        if out_v < self.limits.near_zero_linear_speed:
            curvature_limit = min(curvature_limit, self.limits.angular_epsilon)
        slew_w = min(max(target_w, low_w), high_w)
        out_w = min(max(target_w, max(low_w, -curvature_limit)), min(high_w, curvature_limit))
        if abs(out_v - target_v) > 1.0e-12:
            events.add("LINEAR_ACCEL_SATURATION")
        if abs(slew_w - target_w) > 1.0e-12:
            events.add("ANGULAR_ACCEL_SATURATION")
        if abs(out_w - slew_w) > 1.0e-12:
            events.add("CURVATURE_SATURATION")

        linear_accel = (out_v - self.previous_v) / dt
        angular_accel = (out_w - self.previous_w) / dt
        self.previous_v = out_v
        self.previous_w = out_w
        if out_v < 0.0:
            self.output_reverse_distance += -out_v * dt
        curvature = abs(out_w / out_v) if out_v >= self.limits.near_zero_linear_speed else 0.0
        self._count(events)
        return GuardSample(
            raw_v, raw_w, out_v, out_w, dt, linear_accel, angular_accel,
            curvature, frozenset(events),
        )

    def _safe_stop(self, raw_v: float, raw_w: float, dt: float, events: set[str]) -> GuardSample:
        """Reject unsafe intent while respecting output acceleration bounds."""
        return self._bounded_command(raw_v, raw_w, 0.0, 0.0, dt, events)
