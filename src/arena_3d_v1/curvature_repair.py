import math, time
from types import SimpleNamespace
import numpy as np
from scipy.optimize import minimize, LinearConstraint
from .semantic_world import wrap

def violating_segments(p, maximum):
    delta = np.diff(p, axis=0)
    length = np.linalg.norm(delta[:, :2], axis=1)
    heading = np.divide(2 * np.sin(abs(wrap(delta[:, 2])) / 2), length, out=np.zeros_like(length), where=length > 1e-08)
    (a, b) = (delta[:-1, :2], delta[1:, :2])
    den = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(a + b, axis=1)
    geometry = np.divide(2 * abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), den, out=np.zeros(len(a)), where=den > 1e-12)
    bends = np.flatnonzero(geometry > maximum + 1e-06)
    steering = np.arctan(0.5 * np.divide(wrap(delta[:, 2]), length, out=np.zeros_like(length), where=length > 1e-08))
    jumps = np.flatnonzero(abs(np.diff(steering)) > math.radians(15) + 1e-06)
    return (np.unique(np.r_[np.flatnonzero(heading > maximum + 1e-06), bends, bends + 1, jumps, jumps + 1]), length)

def fit_connector(start, goal, word, params, radius, left_k, right_k, deadline, maximum):
    """Integrate forward circular steps with bounded changes of curvature.

    Dubins words initialize a small constrained fit. The fit changes actual
    positions and headings; steering is later derived without clipping.
    """
    lengths = np.asarray(params) * radius
    length = float(sum(lengths))
    n = max(8, math.ceil(length / 0.05))
    ds = length / n
    if n > 128:
        raise ValueError('CONNECTOR_STATE_BUDGET')
    bins = np.arange(n) * ds
    seed = np.zeros(n)
    for (j, kind) in enumerate(word):
        a = float(sum(lengths[:j]))
        b = a + lengths[j]
        seed += np.maximum(0, np.minimum(bins + ds, b) - np.maximum(bins, a)) / ds * (0 if kind == 'S' else (1 if kind == 'L' else -1) / radius)
    winding = sum(((0 if kind == 'S' else 1 if kind == 'L' else -1) * par for (kind, par) in zip(word, params)))

    def coords(k):
        delta = k * ds
        mid = start[2] + np.cumsum(delta) - delta / 2
        q = delta / 2
        amplitude = ds * np.sinc(q / np.pi)
        return (amplitude * np.cos(mid), amplitude * np.sin(mid), mid, q)

    def eq(k):
        (dx, dy, _, _) = coords(k)
        return np.array([start[0] + sum(dx) - goal[0], start[1] + sum(dy) - goal[1], sum(k) * ds - winding])

    def jac(k):
        (dx, dy, mid, q) = coords(k)
        deriv = -q / 3
        nz = abs(q) > 1e-06
        deriv[nz] = (q[nz] * np.cos(q[nz]) - np.sin(q[nz])) / q[nz] ** 2
        aprime = ds * 0.5 * deriv
        return np.array([ds * (aprime * np.cos(mid) - dy / 2 - (np.cumsum(dy[::-1])[::-1] - dy)), ds * (aprime * np.sin(mid) + dx / 2 + (np.cumsum(dx[::-1])[::-1] - dx)), np.full(n, ds)])

    def objective(k):
        d = np.diff(np.r_[left_k, k, right_k])
        return float(np.sum((k - seed) ** 2) + 4 * np.sum(d * d))

    def gradient(k):
        return 2 * (k - seed) + 8 * (2 * k - np.r_[left_k, k[:-1]] - np.r_[k[1:], right_k])

    def check_budget(_=None):
        if time.monotonic() >= deadline:
            raise TimeoutError('LOCAL_CURVATURE_BUDGET')
    limit = maximum - 0.01
    bounds = [(-limit, limit)] * n
    bounds[0] = (max(-limit, left_k - 0.4), min(limit, left_k + 0.4))
    bounds[-1] = (max(-limit, right_k - 0.4), min(limit, right_k + 0.4))
    if any((lo > hi for (lo, hi) in bounds)):
        raise ValueError('CONNECTOR_BOUNDARY_CURVATURE')

    def feasible(k):
        return np.all(np.isfinite(k)) and np.max(abs(eq(k))) <= 1e-07 and (np.max(abs(np.diff(k))) <= 0.4 + 1e-06) and all((lo - 1e-06 <= v <= hi + 1e-06 for (v, (lo, hi)) in zip(k, bounds)))

    class FeasibleIterate(Exception):

        def __init__(self, k):
            self.k = k.copy()
    iterations = 0

    def on_iteration(k):
        nonlocal iterations
        iterations += 1
        check_budget()
        if feasible(k):
            raise FeasibleIterate(k)
    check_budget()
    try:
        sol = minimize(objective, np.clip(seed, -limit, limit), jac=gradient, bounds=bounds, constraints=[{'type': 'eq', 'fun': eq, 'jac': jac}, LinearConstraint(np.diff(np.eye(n), axis=0), -0.4, 0.4)], method='SLSQP', callback=on_iteration, options={'maxiter': 80, 'ftol': 1e-08})
    except FeasibleIterate as found:
        sol = SimpleNamespace(x=found.k, success=False, nit=iterations, message='Feasible iterate; objective optimization stopped')
    check_budget()
    if not feasible(sol.x):
        raise ValueError('CONNECTOR_FIT_FAILED')
    (dx, dy, _, _) = coords(sol.x)
    p = np.column_stack([start[0] + np.r_[0, np.cumsum(dx)], start[1] + np.r_[0, np.cumsum(dy)], wrap(start[2] + np.r_[0, np.cumsum(sol.x * ds)])])
    p[0] = start
    p[-1] = goal
    return (p, {'word': word, 'initial_radius_m': radius, 'states': n, 'iterations': int(sol.nit), 'optimizer_converged': bool(sol.success), 'optimizer_status': str(sol.message), 'acceptance': 'independently_feasible_not_claimed_optimal', 'endpoint_residual': float(np.max(abs(eq(sol.x)))), 'max_curvature_step': float(max(abs(np.diff(sol.x))))})

def truthful_points(p, original, first, last):
    delta = np.diff(p, axis=0)
    length = np.linalg.norm(delta[:, :2], axis=1)
    steering = np.arctan(0.5 * np.divide(wrap(delta[:, 2]), length, out=np.zeros_like(length), where=length > 1e-08))
    points = []
    for (i, (x, y, yaw)) in enumerate(p):
        inserted = first <= i <= last
        points.append({'x': float(x), 'y': float(y), 'yaw': float(yaw), 'steering': float(steering[min(i, len(steering) - 1)]), 'velocity': 0.0, 'motion_direction': 'forward', 'source': 'l3_local_curvature_repair' if inserted else original.source, 'planner_backend': 'bounded_forward_curvature_fit' if inserted else original.planner_backend, 'backend_version': 'local-curvature-v1' if inserted else original.backend_version})
    return points
