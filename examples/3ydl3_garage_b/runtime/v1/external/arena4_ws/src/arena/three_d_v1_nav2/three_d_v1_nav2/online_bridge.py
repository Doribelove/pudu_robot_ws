"""Online frozen-r1 planner; the production L3 session owns a separate domain."""

from collections import deque
import csv
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav_msgs.srv import GetPlan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

from arena_3d_v1.production_l1 import DeterministicGraphAStarL1
from arena_3d_v1.r1_pipeline import Layered3DV1R1Controller
from arena_3d_v1.pipeline import ProductionL3Adapter
from arena_evaluation import path_audit, topology
from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import two_layer_v1_r1_cache_benchmark as profile
from arena_evaluation import two_layer_v2_semantic_benchmark as semantic_runtime
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.semantic_query_defaults import load_query_set

from .contracts import (QUERY_SET, EXPECTED_MAP_SHA256, EXPECTED_SEMANTIC_MAP_HASH,
                        verify_run_inputs, sha256_file)
from .dynamic_scenarios import _jsonable, _step_trace
from .runtime_safety import SweptFootprintMonitor


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OnlineBridge(Node):
    def __init__(self):
        super().__init__('three_d_v1_online_bridge')
        self.declare_parameter('run_dir', '')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('l3_domain_id', 219)
        root = Path(str(self.get_parameter('run_dir').value))
        self.output = Path(str(self.get_parameter('output_dir').value)) / 'online_planner'
        self.output.mkdir(parents=True, exist_ok=False)
        verify_run_inputs(root)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stop_pub = self.create_publisher(Bool, '/three_d_v1/dynamic_stop', qos)
        self.ready_pub = self.create_publisher(Bool, '/three_d_v1/online_ready', qos)
        self.audit_pub = self.create_publisher(String, '/three_d_v1/online_audit', 10)
        self.trace_pub = self.create_publisher(String, '/three_d_v1/global_layer_trace', 10)
        self.stop_pub.publish(Bool(data=True))
        self.pending = deque()
        self.input_failure = ''
        self.controller = None
        self.query_id = ''
        self.cached = None
        self.sequence = 0
        self.session = None
        self.motion = None
        self.stopped_since = None
        self.create_subscription(Odometry, '/model/jackal/odometry', self.odometry, 10,
                                 callback_group=ReentrantCallbackGroup())
        map_yaml = root / 'derived_map/extracted/optemap.yaml'
        self.ctx = semantic_runtime._context(map_yaml)
        self.sweep = SweptFootprintMonitor(self.ctx.hospital_map)
        topo_dir = root / 'derived_map/topology_cache'
        artifact = topology.load_topology(topo_dir, self.ctx.hospital_map, runtime.FOOTPRINT,
                                          padding_m=.05, safety_margin_m=.05, allow_unknown=False)
        self.l1 = DeterministicGraphAStarL1(self.ctx, artifact, map_hash=self.ctx.map_sha256,
                                           topology_hash=sha256_file(topo_dir / 'topology_graph.json'))
        self.queries, _, _ = load_query_set(QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
                                            actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
                                            require_default_contract=True)
        self.auditor = path_audit.PathAuditor(self.ctx, source_commit='frozen-r1-live-adapter')
        self.spec = runtime.backend_availability()['hybrid_astar']
        if not self.spec.available:
            raise RuntimeError(self.spec.reason)
        # The bridge node's default Context has already captured the main
        # domain. SmacSession creates its own Context and child process here.
        main_domain = os.environ.get('ROS_DOMAIN_ID', '0')
        l3_domain = str(self.get_parameter('l3_domain_id').value)
        if l3_domain == main_domain:
            raise RuntimeError('L3 domain must differ from the main Nav2 domain')
        os.environ['ROS_DOMAIN_ID'] = l3_domain
        self.session = production.SmacSession(
            self.ctx, self.output / 'l3', map_yaml=map_yaml, log_tag='online_frozen_r1',
            local_mask_updates=True, optimization_profile=profile.OPTIMIZATION_PROFILE,
            smac_parameter_profile=profile.SMAC_PARAMETER_PROFILE,
            optimization_stage=profile.OPTIMIZATION_STAGE, enable_mask_reuse_noop=True,
            planner_parameter_overrides={'angle_quantization_bins': 48}, costmap_ack_timeout_s=3.)
        self.session.local_map_update_strategy = 'roi_ack'
        self.session.full_grid_settle_cycles = 0
        try:
            self.session.start()
        except Exception:
            self.session.close()
            raise
        finally:
            os.environ['ROS_DOMAIN_ID'] = main_domain
        # A plan callback may spend several seconds waiting for the measured
        # dynamic stop. Keep snapshot delivery in a separate callback group so
        # the confirming observation can join the same stopped planning pass.
        self.snapshot_callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/three_d_v1/dynamic_snapshot', self.snapshot, 32,
            callback_group=self.snapshot_callback_group)
        self.create_service(GetPlan, '/three_d_v1/compute_plan', self.plan)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info('Online frozen r1 ready; real L3 in domain ' + l3_domain)

    def odometry(self, message):
        now = time.monotonic()
        twist = message.twist.twist
        if abs(twist.linear.x) <= .002 and abs(twist.angular.z) <= .003:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None
        self.motion = (now, message.pose.pose)

    def stopped_pose(self):
        deadline = time.monotonic() + 5.
        while self.context.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            motion = self.motion
            stopped = self.stopped_since
            if motion and now - motion[0] < .5 and stopped is not None and now - stopped >= .5:
                return motion[1]
            time.sleep(.02)
        raise RuntimeError('DYNAMIC_STOP_NOT_CONFIRMED')

    def snapshot(self, message):
        self.stop_pub.publish(Bool(data=True))
        try:
            payload = json.loads(message.data)
            if payload['map_version'] != EXPECTED_MAP_SHA256 or tuple(payload['map_shape']) != self.ctx.hospital_map.occupancy.shape:
                raise ValueError('snapshot map contract mismatch')
            snapshot = DynamicSnapshot.from_cells(
                payload['snapshot_id'], payload['occupied_cells'], timestamp=payload['timestamp'],
                confidence=payload.get('obstacle_confidence'), ttl=payload.get('ttl'),
                map_version=payload['map_version'], map_shape=tuple(payload['map_shape']))
            if payload.get('snapshot_hash', snapshot.snapshot_hash) != snapshot.snapshot_hash:
                raise ValueError('snapshot hash mismatch')
            if len(self.pending) >= 32:
                raise ValueError('snapshot queue overflow')
            self.pending.append(snapshot)
        except (ValueError, KeyError, TypeError) as error:
            self.input_failure = 'DYNAMIC_SNAPSHOT_REJECTED:' + str(error)

    def plan(self, request, response):
        started = time.monotonic_ns()
        self.sequence += 1
        trace = {'request_sequence': self.sequence, 'execution_mode': 'online_r1', 'steps': []}
        try:
            if self.input_failure:
                raise RuntimeError(self.input_failure)
            if request.start.header.frame_id != 'map' or request.goal.header.frame_id != 'map':
                raise RuntimeError('INVALID_FRAME')
            goal = request.goal.pose
            matches = [q for q in self.queries if math.hypot(goal.position.x - q.goal[0], goal.position.y - q.goal[1]) < 1e-6 and
                       abs(math.atan2(math.sin(yaw(goal) - q.goal[2]), math.cos(yaw(goal) - q.goal[2]))) < 1e-6]
            if len(matches) != 1:
                raise RuntimeError('GOAL_NOT_FROZEN_QUERY')
            query = matches[0]
            trace['query_id'] = query.query_id
            new_query = query.query_id != self.query_id
            planning_pose = request.start.pose
            if new_query or self.pending or self.cached is None:
                self.stop_pub.publish(Bool(data=True))
                stop_started = time.monotonic_ns()
                planning_pose = self.stopped_pose()
                trace['stop_confirmed'] = True
                trace['stop_wait_ms'] = (time.monotonic_ns() - stop_started) / 1e6
                trace['stopped_start'] = [planning_pose.position.x, planning_pose.position.y, yaw(planning_pose)]
            need_l3 = new_query
            last_step = None
            if new_query:
                if self.controller:
                    self.controller.lifecycle.clear()
                self.cached = None
                l1_start = time.monotonic_ns()
                initial = self.l1.plan(query)
                trace['l1_ms'] = (time.monotonic_ns() - l1_start) / 1e6
                if initial is None:
                    raise RuntimeError('L1_NO_ROUTE')
                self.controller = Layered3DV1R1Controller(initial, cache_root=self.output / 'l2_cache',
                                                         max_active_states=1, verify_l2_oracle=True)
                l2 = self.controller.initial_l2_result
                if not l2.success or l2.partial_dstar_result_returned:
                    raise RuntimeError(l2.failure_code or 'L2_INITIAL_FAILED')
                trace['l2_ms'] = l2.response_ms
                trace['l2_backend'] = l2.selected_backend
                self.query_id = query.query_id
                self.session.reset_query_state(query.query_id, restore_base_map=False)
            while self.pending:
                snapshot = self.pending.popleft()
                last_step = self.controller.process_snapshot(snapshot, now=time.time(),
                    l1_replan=lambda blocked: self.l1.plan(query, blocked))
                trace['steps'].append(_step_trace(last_step))
                if last_step.l2_result:
                    trace['l2_backend'] = last_step.l2_result.selected_backend
                    trace['l2_ms'] = last_step.l2_result.response_ms
                if last_step.failure_code:
                    self.cached = None
                    raise RuntimeError(last_step.failure_code)
                need_l3 |= last_step.l3_required
            if need_l3:
                current = planning_pose
                start = (current.position.x, current.position.y, yaw(current))
                live_query = replace(query, start=start)
                l3_start = time.monotonic_ns()
                if last_step is not None and last_step.l3_required:
                    outcome = ProductionL3Adapter(self.controller, self.auditor).plan(
                        last_step, live_query, self.session, self.spec)
                    if not outcome.get('success'):
                        raise RuntimeError(outcome.get('failure_code', 'L3_FAILED'))
                    result = outcome['result']
                    audit = result.path_audit
                else:
                    result = self.session.plan(live_query, self.spec, source='3d_v1_r1_online_l3',
                        allowed_mask=self.controller._target_mask(), window_start_index=0,
                        window_end_index=-1, window_path_length_m=0., skip_path_mask_validation=True)
                    if not result.planner_success or not result.points:
                        raise RuntimeError(result.failure_code or 'L3_FAILED')
                    audit = self.auditor.audit(live_query, result.points, self.controller._target_mask())
                    result.path_audit = audit
                trace['l3_ms'] = (time.monotonic_ns() - l3_start) / 1e6
                diagnostics = result.diagnostics or {}
                if diagnostics.get('costmap_update_acknowledged') is not True or diagnostics.get('costmap_ack_mismatch_cells', 0) != 0:
                    raise RuntimeError('ROI_CONTENT_ACK_FAILED')
                if not audit.final_valid_success:
                    raise RuntimeError(audit.metrics.get('failure_code') or 'CANONICAL_AUDIT_FAILED')
                self.sweep.previous = None
                for point in result.points:
                    if self.sweep.observe((point['x'], point['y'], point['yaw'])):
                        raise RuntimeError('PADDED_FOOTPRINT_AUDIT_FAILED')
                self.cached = dict(points=result.points, canonical_path_hash=audit.path_hash,
                                   canonical_audit=audit.metrics, l3_diagnostics=diagnostics)
                self.cached_stamp = self.get_clock().now().to_msg()
                path_file = self.output / f'{self.sequence:06d}_path.csv'
                with path_file.open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['x', 'y', 'yaw'])
                    writer.writerows([[format(float(p[k]), '.17g') for k in ('x', 'y', 'yaw')]
                                      for p in result.points])
                self.cached['path_sha256'] = sha256_file(path_file)
                self.cached['path_file'] = str(path_file)
            if not self.cached:
                raise RuntimeError('NO_AUDITED_PLAN')
            trace.update(query_id=query.query_id, success=True, l1='deterministic_graph_astar',
                         l2=trace.get('l2_backend', 'persistent_state_reused'), l3='smac_hybrid_astar',
                         angle_bins=48, motion_model='DUBIN', cached_path_reused=not need_l3,
                         route_edge_ids=list(self.controller.plan.route_edge_ids),
                         l2_binding_hash=self.controller.l2.binding_hash, canonical_path_audit_reused=True,
                         **self.cached)
            message = NavPath()
            message.header.frame_id = 'map'
            message.header.stamp = self.cached_stamp
            for point in self.cached['points']:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = float(point['x'])
                pose.pose.position.y = float(point['y'])
                pose.pose.orientation.z = math.sin(float(point['yaw']) / 2)
                pose.pose.orientation.w = math.cos(float(point['yaw']) / 2)
                message.poses.append(pose)
            response.plan = message
            self.audit_pub.publish(String(data=json.dumps(_jsonable(trace))))
            self.stop_pub.publish(Bool(data=False))
        except Exception as error:
            self.stop_pub.publish(Bool(data=True))
            trace.update(success=False, failure_code=str(error))
            self.get_logger().error(str(error))
        trace['request_ms'] = (time.monotonic_ns() - started) / 1e6
        payload = json.dumps(_jsonable(trace), allow_nan=False)
        self.trace_pub.publish(String(data=payload))
        (self.output / f'{self.sequence:06d}.json').write_text(payload + '\n')
        return response

    def close(self):
        if self.context.ok():
            self.stop_pub.publish(Bool(data=True))
        if self.controller:
            self.controller.lifecycle.clear()
        if self.session:
            self.session.close()


def main():
    rclpy.init()
    node = None
    try:
        node = OnlineBridge()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
