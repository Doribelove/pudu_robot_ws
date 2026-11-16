"""Weak lifecycle callback; controller callbacks must not own the controller."""
import weakref
from types import SimpleNamespace
class WeakClear:
    def __init__(self,method):self.method=weakref.WeakMethod(method)
    def __call__(self):
        method=self.method()
        if method is not None:return method()
def lifecycle(clear):return SimpleNamespace(clear=WeakClear(clear))
