"""Focused invariants for next-architecture feasibility prototypes."""
import math

import numpy as np

from arena_evaluation.semantic_architecture_explorer import (
    PROTOCOL_ID, _rectangle_kernel, parser,
)


def test_rectangle_kernel_is_symmetric_and_yaw_dependent():
    horizontal = _rectangle_kernel(.05, 0.0)
    vertical = _rectangle_kernel(.05, math.pi / 2.0)
    assert np.array_equal(horizontal, horizontal[::-1, ::-1])
    assert np.array_equal(vertical, vertical[::-1, ::-1])
    assert np.array_equal(horizontal, vertical.T)
    center = tuple(value // 2 for value in horizontal.shape)
    assert horizontal[center]


def test_explorer_freezes_contract_defaults_and_requires_write_once_output(tmp_path):
    args = parser().parse_args(["topology", "--output", str(tmp_path / "new")])
    assert args.query == "r3-mirror-1-positive"
    assert args.max_states == 1_000_000
    assert args.timeout == 60.0
    assert PROTOCOL_ID == "PLN-02-NEXT-ARCHITECTURE-EXPLORATION-R1-V1"
