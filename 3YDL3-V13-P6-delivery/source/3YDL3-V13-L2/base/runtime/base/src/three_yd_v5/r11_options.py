import os
VALID={'control':frozenset(),'A':frozenset('A'),'B':frozenset('B'),'AB':frozenset('AB'),'ABC':frozenset('ABC'),'C':frozenset('C')}
def mode():
    value=os.environ.get('THREEYD_R11_MODE','AB')
    if value not in VALID:raise ValueError('Unknown r11 mode '+value)
    return value
def enabled(item):return item in VALID[mode()]
