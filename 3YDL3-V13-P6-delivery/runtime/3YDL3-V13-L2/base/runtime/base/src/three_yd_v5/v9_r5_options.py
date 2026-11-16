import os
def enabled():
 value=os.environ.get('THREEYD_V9_R5_MODE','control')
 if value not in ('control','C'):raise ValueError('invalid V9-r5 mode: '+value)
 return value=='C'
