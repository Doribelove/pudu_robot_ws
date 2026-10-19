"""Independent V10-r5 controls; experiments are opt-in."""
import os


def mode():
    return os.environ.get('THREEYD_V10_R5_MODE', 'control')


def enabled(flag):
    return mode() != 'control' and flag in mode()
