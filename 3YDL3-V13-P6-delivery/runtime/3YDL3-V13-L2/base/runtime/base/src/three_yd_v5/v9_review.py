"""Independent V9-r1 review controls: storage, joints, cache preparation."""
import os

VALID=('control','S','J','K','SJ','SK','JK','SJK')

def mode():
    value=os.environ.get('THREEYD_V9_R1_MODE','control')
    if value not in VALID:raise ValueError('INVALID_V9_R1_MODE:'+value)
    return value

def enabled(feature):
    value=mode()
    return value!='control' and feature in value
