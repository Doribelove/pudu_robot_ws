"""Load the byte-for-byte r2 L1/L3 module without replacing current research files."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

from arena_evaluation import endpoint_heading
from arena_evaluation import two_layer_v1_formal_benchmark as parent
from arena_evaluation import two_layer_v1_r1_cache_benchmark as r1
from arena_evaluation import two_layer_v1_r2_roi_pathaudit_benchmark as r2

from .contracts import SNAPSHOT


FROZEN_CANDIDATE_PATH = SNAPSHOT / (
    "external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/"
    "l1_l3_corridor_hybrid_smoke.py"
)
MODULE_NAME = "arena_evaluation._frozen_2a_v1_r2_l1_l3_corridor_hybrid_smoke"


def _load():
    existing = sys.modules.get(MODULE_NAME)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(MODULE_NAME, FROZEN_CANDIDATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load frozen 2A module: {FROZEN_CANDIDATE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


candidate = _load()

# The delivered r1/r2 wrappers bind their candidate module at import time.
# Redirect only those module globals to the exact archived implementation.
parent.candidate = candidate
r1.candidate = candidate
r2.candidate = candidate
endpoint_heading.candidate = candidate

__all__ = ["candidate", "endpoint_heading", "parent", "r1", "r2"]
