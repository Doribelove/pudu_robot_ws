import os

def mode():return os.environ.get("THREEYD_V10_R6_MODE","control")
def enabled(flag):return mode() != "control" and flag in mode()
