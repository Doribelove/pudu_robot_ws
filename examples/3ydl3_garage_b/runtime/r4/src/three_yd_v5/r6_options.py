import os
def mode():
    value=os.environ.get('THREEYD_V8_R6_MODE','control')
    if value not in ('control','A','AB','ABC'):raise ValueError('invalid r6 mode '+value)
    return value

def enabled(part):return part in mode() if mode()!='control' else False
