"""Predeclared relative-quality check; no change to any planner safety audit.

Clearance samples come from the original float32 distance field. Record the
unadjusted comparison as well as a half-ULP input precision bound, capped 1um.
A non-float32 value never silently receives this allowance.
"""
import math
import numpy as np


def half_ulp(value):
    v = np.float32(value)
    if not math.isfinite(value) or float(v) != value or value < 0:
        raise ValueError('clearance is not an original finite float32 sample')
    above = float(np.nextafter(v, np.float32(np.inf))) - float(v)
    below = float(v) - float(np.nextafter(v, np.float32(-np.inf)))
    return max(above, below) / 2


def quality(before_length, after_length, before_clearance, after_clearance):
    if not (math.isfinite(before_length) and math.isfinite(after_length)
            and before_length > 0 and after_length > 0):
        raise ValueError('invalid complete trajectory lengths')
    allowance = min(1e-6, half_ulp(before_clearance) + half_ulp(after_clearance))
    ratio = after_length / before_length
    delta = after_clearance - before_clearance
    direct = ratio <= 1.02 and delta >= -.05
    precise = ratio <= 1.02 and delta >= -.05 - allowance
    return {'length_ratio': ratio, 'minimum_clearance_before_m': before_clearance,
            'minimum_clearance_after_m': after_clearance,
            'minimum_clearance_delta_m': delta,
            'float32_input_precision_allowance_m': allowance,
            'quality_gate_direct': direct, 'quality_gate': precise,
            'precision_rule': 'half-ULP of each original float32 input, sum capped 1e-6 m'}
