"""Measured motion and telemetry gates used before mission advancement."""

import math
from dataclasses import dataclass, field

from .guard_core import GuardLimits
from .contracts import FOOTPRINT, FOOTPRINT_PADDING_M


@dataclass
class MotionEvidence:
    limits: GuardLimits = field(default_factory=GuardLimits)
    samples: int = 0
    reverse_distance: float = 0.0
    rotate_samples: int = 0
    curvature_violations: int = 0
    speed_violations: int = 0
    accel_violations: int = 0
    invalid_samples: int = 0
    collision_samples: int = 0
    previous: tuple | None = None

    def observe(self, stamp_ns, v, w, *, collision=False):
        if not all(math.isfinite(x) for x in (stamp_ns, v, w)):
            self.invalid_samples += 1
            return
        self.samples += 1
        self.collision_samples += int(collision)
        epsilon = 1e-8
        self.rotate_samples += int(abs(v) < self.limits.near_zero_linear_speed and
                                   abs(w) > self.limits.angular_epsilon)
        self.curvature_violations += int(abs(v) >= self.limits.near_zero_linear_speed and
                                        abs(w) > self.limits.maximum_curvature * abs(v) + epsilon)
        self.speed_violations += int(v < -epsilon or v > self.limits.max_linear_speed + epsilon or
                                    abs(w) > self.limits.max_angular_speed + epsilon)
        if self.previous is not None:
            previous_stamp, previous_v, previous_w = self.previous
            dt = (stamp_ns - previous_stamp) / 1e9
            if dt <= 0:
                self.invalid_samples += 1
            else:
                self.reverse_distance += max(0., -v) * dt
                self.accel_violations += int(
                    abs(v - previous_v) > self.limits.max_linear_accel * dt + epsilon or
                    abs(w - previous_w) > self.limits.max_angular_accel * dt + epsilon)
        self.previous = (stamp_ns, v, w)

    @property
    def failure_code(self):
        for count, code in (
            (self.invalid_samples, "INVALID_MOTION_SAMPLE"),
            (self.collision_samples, "MEASURED_FOOTPRINT_COLLISION"),
            (self.speed_violations, "MEASURED_SPEED_OR_REVERSE_VIOLATION"),
            (self.rotate_samples, "MEASURED_ROTATE_IN_PLACE"),
            (self.curvature_violations, "MEASURED_CURVATURE_VIOLATION"),
            (self.accel_violations, "MEASURED_ACCELERATION_VIOLATION"),
        ):
            if count:
                return code
        return ""


def telemetry_failure(payload, *, controller=False):
    if controller:
        if payload.get("feasibility_stop") or payload.get("collision_stop"):
            return "TEB_FEASIBILITY_STOP"
        if payload.get("planner_success") is not True or payload.get("diverged"):
            return "TEB_PLANNING_FAILED"
    else:
        events = str(payload.get("events", "")).split(";")
        for event in events:
            if "HARD_STOP" in event or event == "COMMAND_WATCHDOG_STOP":
                return event
        limits = GuardLimits()
        for name, limit in (("output_linear_accel", limits.max_linear_accel),
                            ("output_angular_accel", limits.max_angular_accel)):
            value = payload.get(name)
            if not isinstance(value, (float, int)) or not math.isfinite(value) or abs(value) > limit + 1e-8:
                return "COMMAND_ACCELERATION_VIOLATION"
    return ""


class SweptFootprintMonitor:
    def __init__(self, grid):
        from scipy.ndimage import distance_transform_edt

        self.grid = grid
        self.distance = distance_transform_edt(grid.occupancy == 0, sampling=grid.resolution)
        self.footprint = [[x + math.copysign(FOOTPRINT_PADDING_M, x),
                           y + math.copysign(FOOTPRINT_PADDING_M, y)] for x, y in FOOTPRINT]
        self.radius = max(math.hypot(x, y) for x, y in self.footprint)
        self.previous = None
        self.minimum_clearance_lower_bound = math.inf

    def observe(self, pose):
        first = self.previous or pose
        self.previous = pose
        dx, dy = pose[0] - first[0], pose[1] - first[1]
        dyaw = math.atan2(math.sin(pose[2] - first[2]), math.cos(pose[2] - first[2]))
        steps = max(1, math.ceil(math.hypot(dx, dy) / .025), math.ceil(abs(dyaw) / math.radians(1)))
        for step in range(steps + 1):
            f = step / steps
            sample = (first[0] + f * dx, first[1] + f * dy, first[2] + f * dyaw)
            cell = self.grid.world_to_cell(*sample[:2])
            if cell is None:
                return True
            boundary = min(sample[0] - self.grid.origin[0], sample[1] - self.grid.origin[1],
                           self.grid.origin[0] + self.grid.width * self.grid.resolution - sample[0],
                           self.grid.origin[1] + self.grid.height * self.grid.resolution - sample[1])
            bound = min(float(self.distance[cell]) - math.sqrt(2) * self.grid.resolution, boundary) - self.radius
            self.minimum_clearance_lower_bound = min(self.minimum_clearance_lower_bound, max(0., bound))
            if bound <= 0 and self.grid.footprint_collision(sample, self.footprint, unknown_is_collision=True):
                return True
        return False
