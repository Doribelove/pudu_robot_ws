"""Independent storage/batch candidates; parent AS remains the default."""
import os


def enabled(flag='N'):
    mode=os.environ.get('THREEYD_V9_R7_MODE','control')
    if mode not in ('control','N','T','NT'):
        raise ValueError('invalid V9-r7 mode: '+mode)
    return flag in mode if mode!='control' else False
