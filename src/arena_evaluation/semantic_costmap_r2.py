from __future__ import annotations
import numpy as np
from . import _nav2_effective_costmap

def pinned_nav2_effective_master(static_cost: np.ndarray, *, resolution: float, inflation_radius_m: float, cost_scaling_factor: float, inscribed_radius_m: float) -> np.ndarray:
    """Run the pinned Humble InflationLayer algorithm in server row order.

    The old SciPy EDT model is mathematically nearest-obstacle exact, while the
    pinned implementation's source propagation and ``seen_`` ordering leaves a
    small, deterministic set of different cells.  Nav2 traverses its costmap
    bottom row first; HospitalMap arrays are top row first, hence both flips.
    """
    source = np.asarray(static_cost, dtype=np.uint8)
    if source.ndim != 2:
        raise ValueError('static_cost must be a 2-D grid')
    server_order = np.ascontiguousarray(np.flipud(source), dtype=np.uint8)
    payload = _nav2_effective_costmap.inflate(server_order, int(server_order.shape[1]), int(server_order.shape[0]), float(resolution), float(inflation_radius_m), float(cost_scaling_factor), float(inscribed_radius_m))
    effective_server = np.frombuffer(payload, dtype=np.uint8).reshape(server_order.shape)
    return np.ascontiguousarray(np.flipud(effective_server), dtype=np.uint8)
