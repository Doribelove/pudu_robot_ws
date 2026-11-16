"""Independent V9 controls; the frozen V8-r11 AB remains in both arms."""
import os

def mode():
    value=os.environ.get('THREEYD_V9_MODE','control')
    if value not in ('control','A','B','C','AB','AC','BC','ABC'):
        raise ValueError('INVALID_V9_MODE:'+value)
    return value

def enabled(letter):
    return letter in mode() if mode()!='control' else False
