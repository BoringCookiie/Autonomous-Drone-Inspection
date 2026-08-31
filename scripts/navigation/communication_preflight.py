#!/usr/bin/env python3
"""Runtime communication gate for the navigation stack.

The node publishes ``/navigation/preflight_ok`` with transient-local,
reliable QoS.  The path follower must see ``true`` before it configures,
arms, or enters OFFBOARD.  A passing check requires the expected ROS types,
non-duplicate publishers, compatible endpoint QoS, live message rates,
non-zero timestamps, a valid TF chain, and a bounded non-empty OctoMap.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Clock


EXPECTED_TYPES = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/uas1/state': 'mavros_msgs/msg/State',
    '/uas1/local_position/pose': 'geometry_msgs/msg/PoseStamped',
    '/uas1/setpoint_position/local': 'geometry_msgs/msg/PoseStamped',
    '/camera/color/image_raw': 'sensor_msgs/msg/Image',
    '/camera/color/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/camera/depth/image_raw': 'sensor_msgs/msg/Image',
    '/points': 'sensor_msgs/msg/PointCloud2',
    '/points_clean': 'sensor_msgs/msg/PointCloud2',
    '/octomap_point_cloud_centers': 'sensor_msgs/msg/PointCloud2',
    '/planned_path': 'nav_msgs/msg/Path',
    '/navigation/goal': 'geometry_msgs/msg/PoseStamped',
    '/path_follower/status': 'std_msgs/msg/String',
    '/navigation/preflight_ok': 'std_msgs/msg/Bool',
}

REQUIRED_GRAPH = {
    '/clock': (1, 0),
    '/uas1/state': (1, 1),
    '/uas1/local_position/pose': (1, 1),
    '/uas1/setpoint_position/local': (1, 1),
    '/camera/color/image_raw': (1, 1),
    '/camera/color/camera_info': (1, 1),
    '/planned_path': (1, 1),
    '/navigation/goal': (0, 1),
    '/path_follower/status': (1, 0),
    '/navigation/preflight_ok': (1, 1),
}


class CommunicationPreflight(Node):
    def __init__(self) -> None:
        super().__init__('communication_preflight')
        self.declare_parameter('require_depth', True)
        self.declare_parameter('sample_window_s', 5.0)
        self.declare_parameter('max_map_points', 200000)
        require_depth = self.get_parameter('require_depth').value
        self.require_depth = (
            require_depth.lower() in ('1', 'true', 'yes')
            if isinstance(require_depth, str)
            else bool(require_depth)
        )
        self.window_s = float(self.get_parameter('sample_window_s').value)
        self.max_map_points = int(self.get_parameter('max_map_points').value)

        self._started = time.monotonic()
        self._last_report = 0.0
        self._passed_once = False
        self._samples: dict[str, list[tuple[float, int, str]]] = {}
        self._map_points = 0
        self._map_frame = ''
        self._map_bounds_ok = False
        self._pose = None
        self._state = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, '/navigation/preflight_ok', latched)
        self._publish(False)

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', lambda m: self._sample('/clock', m, 'clock'), qos_be)
        self.create_subscription(State, '/uas1/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/uas1/local_position/pose', self._pose_cb, qos_be)
        self.create_subscription(Image, '/camera/color/image_raw', lambda m: self._sample('/camera/color/image_raw', m, 'image'), 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: self._sample('/camera/color/camera_info', m, 'camera_info'), 10)
        self.create_subscription(Path, '/planned_path', lambda m: self._sample('/planned_path', m, 'path'), 10)
        if self.require_depth:
            self.create_subscription(PointCloud2, '/points', lambda m: self._sample('/points', m, 'cloud'), 10)
            self.create_subscription(PointCloud2, '/points_clean', self._clean_cloud_cb, 10)
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self._map_cb, map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self._check)
        self.get_logger().info(
            f'Communication preflight started (depth_required={self.require_depth}, '
            f'window={self.window_s:.1f}s).'
        )

    def _publish(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.ready_pub.publish(msg)

    @staticmethod
    def _stamp(msg) -> int:
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is None:
            stamp = getattr(msg, 'clock', None)
        if stamp is None:
            return 0
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _sample(self, topic: str, msg, kind: str) -> None:
        now = time.monotonic()
        self._samples.setdefault(topic, []).append((now, self._stamp(msg), kind))
        cutoff = now - max(self.window_s * 2.0, 10.0)
        self._samples[topic] = [s for s in self._samples[topic] if s[0] >= cutoff]

    def _state_cb(self, msg: State) -> None:
        self._state = msg
        self._sample('/uas1/state', msg, 'state')

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._pose = msg
        self._sample('/uas1/local_position/pose', msg, 'pose')

    def _clean_cloud_cb(self, msg: PointCloud2) -> None:
        self._sample('/points_clean', msg, 'cloud')
        if not msg.header.frame_id:
            return

    def _map_cb(self, msg: PointCloud2) -> None:
        self._sample('/octomap_point_cloud_centers', msg, 'map')
        self._map_frame = msg.header.frame_id
        self._map_points = int(msg.width) * int(msg.height)
        self._map_bounds_ok = False
        if self._map_points == 0 or self._map_points > self.max_map_points:
            return
        try:
            pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            count = 0
            for point in pts:
                if not all(math.isfinite(float(v)) and abs(float(v)) <= 50.0 for v in point):
                    return
                count += 1
            self._map_bounds_ok = count > 0
        except Exception:
            self._map_bounds_ok = False

    def _topic_types(self) -> dict[str, set[str]]:
        return {
            name: set(types)
            for name, types in self.get_topic_names_and_types()
        }

    @staticmethod
    def _qos_compatible(pub, sub) -> bool:
        pub_rel = pub.qos_profile.reliability
        sub_rel = sub.qos_profile.reliability
        if pub_rel == ReliabilityPolicy.BEST_EFFORT and sub_rel == ReliabilityPolicy.RELIABLE:
            return False
        pub_dur = pub.qos_profile.durability
        sub_dur = sub.qos_profile.durability
        if pub_dur == DurabilityPolicy.VOLATILE and sub_dur == DurabilityPolicy.TRANSIENT_LOCAL:
            return False
        return True

    def _graph_checks(self) -> list[str]:
        failures: list[str] = []
        topic_types = self._topic_types()
        required = dict(REQUIRED_GRAPH)
        if self.require_depth:
            required.update({
                '/points': (1, 1),
                '/points_clean': (1, 1),
                '/octomap_point_cloud_centers': (1, 1),
            })
        for topic, expected in EXPECTED_TYPES.items():
            if topic in required and topic_types.get(topic) != {expected}:
                failures.append(f'{topic}: expected type {expected}, got {sorted(topic_types.get(topic, set()))}')
        for topic, (pub_min, sub_min) in required.items():
            pubs = self.get_publishers_info_by_topic(topic)
            subs = self.get_subscriptions_info_by_topic(topic)
            if len(pubs) < pub_min:
                failures.append(f'{topic}: publishers {len(pubs)} < {pub_min}')
            if len(subs) < sub_min:
                failures.append(f'{topic}: subscribers {len(subs)} < {sub_min}')
            if pub_min == 1 and len(pubs) != 1:
                failures.append(f'{topic}: duplicate publishers ({len(pubs)})')
            for pub in pubs:
                for sub in subs:
                    if not self._qos_compatible(pub, sub):
                        failures.append(f'{topic}: incompatible QoS {pub.node_name}->{sub.node_name}')
        return failures

    def _rate_failures(self) -> list[str]:
        failures: list[str] = []
        required_rates = {
            '/clock': 1.0,
            # State is published at approximately 1 Hz by MAVROS; a five
            # sample window can legitimately contain only four samples.
            '/uas1/state': 0.5,
            '/uas1/local_position/pose': 5.0,
            '/camera/color/image_raw': 2.0,
            '/camera/color/camera_info': 1.0,
        }
        if self.require_depth:
            required_rates.update({
                '/points': 1.0,
                '/points_clean': 1.0,
                '/octomap_point_cloud_centers': 0.1,
            })
        now = time.monotonic()
        for topic, minimum in required_rates.items():
            samples = [s for s in self._samples.get(topic, []) if s[0] >= now - self.window_s]
            rate = len(samples) / self.window_s
            if len(samples) < 2 or rate < minimum:
                failures.append(f'{topic}: rate {rate:.2f} Hz < {minimum:.2f} Hz')
            if samples and any(stamp == 0 for _, stamp, _ in samples):
                failures.append(f'{topic}: zero timestamp')
        return failures

    def _tf_failures(self) -> list[str]:
        failures = []
        frames = ['depth_camera']
        if self.require_depth:
            frames.append('x500_depth_0/rgbd_camera_link/rgbd_camera')
        for child in frames:
            try:
                if not self.tf_buffer.can_transform('odom', child, rclpy.time.Time()):
                    failures.append(f'TF odom->{child} unavailable')
            except TransformException:
                failures.append(f'TF odom->{child} unavailable')
        return failures

    def _check(self) -> None:
        # This is an arming gate, not a flight controller. Once the complete
        # graph has passed, keep the latch true for the lifetime of this
        # simulator run; a transient sample-window boundary must not revoke a
        # valid startup decision.
        if self._passed_once:
            self._publish(True)
            return
        failures = self._graph_checks()
        warmup = time.monotonic() - self._started < self.window_s
        if warmup:
            failures.append('warming up live streams')
        failures.extend(self._rate_failures())
        failures.extend(self._tf_failures())
        if self.require_depth:
            if self._map_frame != 'odom':
                failures.append(f'OctoMap frame is {self._map_frame!r}, expected odom')
            if not self._map_bounds_ok:
                failures.append(
                    f'OctoMap invalid/empty/beyond bounds (points={self._map_points})'
                )
        passed = not failures
        self._publish(passed)
        if passed and not self._passed_once:
            self._passed_once = True
            self.get_logger().info('COMMUNICATION PREFLIGHT PASSED. Vehicle control is unlocked.')
        elif not passed and time.monotonic() - self._last_report > 5.0:
            self._last_report = time.monotonic()
            self.get_logger().warn('Preflight blocked: ' + '; '.join(failures[:8]))


def main() -> None:
    rclpy.init()
    node = CommunicationPreflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
