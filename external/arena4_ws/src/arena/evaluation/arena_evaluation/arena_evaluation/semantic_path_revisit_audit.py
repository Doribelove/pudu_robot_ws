"""Fail-closed loop screening, independent of semantic sample density.

A geometric revisit is not a continuous-space infeasibility proof, nor proof
of intentional metric manipulation. It requires a detour justification before
the path may be used to pass the no-unnecessary-loop acceptance condition.
"""
from __future__ import annotations

import numpy as np


def audit_revisits(path, minimum_separation_m=.5):
    xy = np.asarray(path, dtype=float)[:, :2]
    if len(xy) < 2 or not np.all(np.isfinite(xy)):
        raise ValueError("a finite path with at least two poses is required")
    vectors = np.diff(xy, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    station = np.r_[0., np.cumsum(lengths)]
    events = []
    overlaps = 0
    for i, vector in enumerate(vectors[:-2]):
        if lengths[i] <= 1e-12:
            continue
        indices = np.arange(i+2, len(vectors))
        other = vectors[indices]
        delta = xy[indices] - xy[i]
        denominator = vector[0]*other[:, 1] - vector[1]*other[:, 0]
        transverse = np.abs(denominator) > 1e-12
        first = np.divide(delta[:, 0]*other[:, 1]-delta[:, 1]*other[:, 0], denominator,
                          out=np.full(len(indices), np.nan), where=transverse)
        second = np.divide(delta[:, 0]*vector[1]-delta[:, 1]*vector[0], denominator,
                           out=np.full(len(indices), np.nan), where=transverse)
        valid = (first >= -1e-9) & (first <= 1+1e-9) & (second >= -1e-9) & (second <= 1+1e-9)
        for k in np.flatnonzero(valid):
            j = indices[k]
            a = station[i] + float(np.clip(first[k], 0, 1))*lengths[i]
            b = station[j] + float(np.clip(second[k], 0, 1))*lengths[j]
            if b-a < minimum_separation_m:
                continue
            if any(abs(e['station_a_m']-a) < .05 and abs(e['station_b_m']-b) < .05 for e in events):
                continue
            events.append({'station_a_m': a, 'station_b_m': b, 'enclosed_length_m': b-a,
                           'xy': (xy[i]+first[k]*vector).tolist()})
        collinear = ~transverse & (np.abs(delta[:, 0]*vector[1]-delta[:, 1]*vector[0]) < 1e-10)
        projection = delta @ vector / lengths[i]**2
        endpoint = projection + other @ vector / lengths[i]**2
        overlap = np.minimum(1., np.maximum(projection, endpoint))-np.maximum(0., np.minimum(projection, endpoint))
        overlaps += int(np.count_nonzero(collinear & (overlap*lengths[i] > 1e-6)
                         & (station[indices]-station[i+1] >= minimum_separation_m)))
    duplicates = int(np.count_nonzero(lengths <= 1e-12))
    passed = not events and not overlaps and not duplicates
    return {'revisit_screen_passed': passed, 'intersection_events': events,
            'nonlocal_collinear_segment_pairs': overlaps, 'zero_translation_steps': duplicates,
            'status': 'NO_REVISIT_DETECTED' if passed else 'DETOUR_JUSTIFICATION_REQUIRED',
            'scope': 'piecewise-linear output geometry; not a proof of intentional manipulation',
            'minimum_nonlocal_separation_m': minimum_separation_m}
