"""Independent V10-r4 experiments; defaults stay disabled for diagnostics."""
import os


def mode():return os.environ.get('THREEYD_V10_R4_MODE','control')


def enabled(flag):return mode()!='control' and flag in mode()
