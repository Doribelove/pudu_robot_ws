"""Independent V9-r2 controls; parent AS remains the reference behavior."""
import os
VALID=('control','D','P','M','DP','DM','PM','DPM')
def mode():
    value=os.environ.get('THREEYD_V9_R2_MODE','control')
    if value not in VALID:raise ValueError('INVALID_V9_R2_MODE:'+value)
    return value
def enabled(feature):
    value=mode();return value!='control' and feature in value
