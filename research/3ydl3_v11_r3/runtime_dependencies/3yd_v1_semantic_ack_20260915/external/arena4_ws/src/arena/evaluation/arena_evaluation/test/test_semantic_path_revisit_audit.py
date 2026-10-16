import numpy as np
import pytest

from arena_evaluation.semantic_path_revisit_audit import audit_revisits


def test_straight_is_not_a_revisit():
    assert audit_revisits([[0, 0], [1, 0], [2, 0], [3, 0]])['revisit_screen_passed']


def test_crossing_is_reported_with_arc_stations():
    result = audit_revisits([[0, 0], [2, 2], [0, 2], [2, 0]])
    assert not result['revisit_screen_passed']
    event = result['intersection_events'][0]
    assert event['xy'] == [1, 1]
    assert event['enclosed_length_m'] == pytest.approx(2+2**.5*2)


def test_subdividing_path_does_not_hide_crossing():
    path = np.array([[0, 0], [2, 2], [0, 2], [2, 0]])
    dense = np.vstack([a+(b-a)*np.linspace(0, 1, 30, endpoint=False)[:, None]
                       for a, b in zip(path, path[1:])] + [path[-1:]])
    result = audit_revisits(dense)
    assert len(result['intersection_events']) == 1
    assert not result['revisit_screen_passed']


def test_retraced_line_and_duplicates_fail_closed():
    assert not audit_revisits([[0, 0], [3, 0], [3, 1], [2, 0], [1, 0]])['revisit_screen_passed']
    assert not audit_revisits([[0, 0], [0, 0], [1, 0]])['revisit_screen_passed']


def test_natural_hairpin_without_revisit_passes():
    assert audit_revisits([[0, 0], [3, 0], [3.5, .5], [3, 1], [0, 1]])['revisit_screen_passed']
