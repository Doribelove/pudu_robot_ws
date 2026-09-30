"""Independent r10 ablations. No navigation contract switches."""
import os
VALID={'control':frozenset(), 'A':frozenset('A'), 'B':frozenset('B'),
       'C':frozenset('C'), 'ABC':frozenset('ABC')}
def mode():
    value=os.environ.get('THREEYD_R10_MODE','ABC')
    if value not in VALID:raise ValueError('Unknown r10 ablation: '+value)
    return value
def enabled(item):return item in VALID[mode()]
