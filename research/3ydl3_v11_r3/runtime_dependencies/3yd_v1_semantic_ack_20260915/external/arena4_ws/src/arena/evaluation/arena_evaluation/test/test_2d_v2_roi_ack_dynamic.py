from types import SimpleNamespace

import numpy as np

from arena_evaluation import layered_2d_v2_pipeline as v2
from arena_evaluation import unified_four_backends_smoke as shared


def test_v2_reuses_shared_ack_and_pathaudit_implementations():
    assert v2.SmacSession is shared.SmacSession
    assert v2.ROI_MAX_MESSAGE_BYTES == 128 * 1024


def test_ack_must_precede_smac_action():
    events = []

    class Session:
        def update_local_mask(self, _mask):
            events.append("ack")
            return {"costmap_update_acknowledged": True,
                    "costmap_ack_mismatch_cells": 0}

        def call_action(self):
            assert events == ["ack"]
            events.append("smac")

    session = Session()
    info = session.update_local_mask(np.ones((2, 2), dtype=bool))
    assert info["costmap_update_acknowledged"]
    session.call_action()
    assert events == ["ack", "smac"]


def test_content_ack_rejects_mismatch_even_when_client_publish_succeeded():
    session = object.__new__(shared.SmacSession)
    session.costmap_ack_timeout_s = 0.01
    session._last_server_update_time_ns = -1
    session._costmap_ack_sequence = 0
    session.client = None
    expected = np.full((3, 3), 100, dtype=np.int8)
    expected[1, 1] = 0
    changed = np.ones_like(expected, dtype=bool)
    server = np.full_like(expected, 254, dtype=np.uint8)
    session._server_costmap_snapshot = lambda _deadline: (server, 10)
    info = session._wait_for_costmap_ack(expected, changed)
    assert info["costmap_update_acknowledged"] is False
    assert info["costmap_ack_mismatch_cells"] > 0
