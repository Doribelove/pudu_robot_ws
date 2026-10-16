from collections import deque
import json
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.srv import GetPlan
from std_msgs.msg import String

from three_d_v1_nav2.online_bridge import OnlineBridge


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def bridge(tmp_path):
    publisher = Publisher()
    node = SimpleNamespace(sequence=0, input_failure='', pending=deque(),
        query_id='frozen', queries=[SimpleNamespace(query_id='frozen', goal=(1., 2., 0.))],
        cached={'points': [{'x': 0., 'y': 0., 'yaw': 0.}, {'x': 1., 'y': 2., 'yaw': 0.}],
                'canonical_path_hash': 'original-audit-hash'},
        cached_stamp=Time(sec=123), output=tmp_path,
        controller=SimpleNamespace(plan=SimpleNamespace(route_edge_ids=(202,)),
                                   l2=SimpleNamespace(binding_hash='original-binding')),
        stop_pub=publisher, audit_pub=Publisher(), trace_pub=Publisher(),
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    return node


def request():
    req = GetPlan.Request()
    req.start.header.frame_id = req.goal.header.frame_id = 'map'
    req.goal.pose.position.x, req.goal.pose.position.y = 1., 2.
    req.goal.pose.orientation.w = 1.
    return req


def test_cached_path_keeps_stamp_and_canonical_identity(tmp_path):
    node = bridge(tmp_path)
    for _ in range(2):
        result = OnlineBridge.plan(node, request(), GetPlan.Response())
        assert result.plan.header.stamp.sec == 123
        assert len(result.plan.poses) == 2
    trace = json.loads((tmp_path / '000002.json').read_text())
    assert trace['cached_path_reused'] is True
    assert trace['canonical_path_hash'] == 'original-audit-hash'
    assert trace['steps'] == []


def test_invalid_frame_returns_empty_path_and_holds_stop(tmp_path):
    node = bridge(tmp_path)
    req = request()
    req.start.header.frame_id = 'odom'
    result = OnlineBridge.plan(node, req, GetPlan.Response())
    assert not result.plan.poses
    assert node.stop_pub.messages[-1].data is True
    trace = json.loads((tmp_path / '000001.json').read_text())
    assert trace['failure_code'] == 'INVALID_FRAME'


def test_malformed_snapshot_latches_failure(tmp_path):
    node = bridge(tmp_path)
    OnlineBridge.snapshot(node, String(data='{'))
    assert node.input_failure.startswith('DYNAMIC_SNAPSHOT_REJECTED:')
    assert not node.pending
    assert node.stop_pub.messages[-1].data is True


def test_stop_requires_fresh_stable_odometry():
    pose = object()
    node = SimpleNamespace(context=SimpleNamespace(ok=lambda: True),
        motion=(time.monotonic(), pose), stopped_since=time.monotonic() - 1.)
    assert OnlineBridge.stopped_pose(node) is pose
    node.context.ok = lambda: False
    try:
        OnlineBridge.stopped_pose(node)
    except RuntimeError as error:
        assert str(error) == 'DYNAMIC_STOP_NOT_CONFIRMED'
    else:
        raise AssertionError('shutdown context accepted stale stop evidence')
