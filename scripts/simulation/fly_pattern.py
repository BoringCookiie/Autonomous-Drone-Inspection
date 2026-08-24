#!/usr/bin/env python3
"""
OFFBOARD demo: push SITL-friendly PX4 params via MAVROS, stream setpoints, arm, fly, AUTO.LAND.

Use MAVROS under /uas1 (launch mavros with -r __ns:=/uas1). A stray /mavros/cmd/arming must
not be used — params would miss the FCU and arming stays strict.

Env: MAVROS_NS (default /uas1; /uas1/mavros is normalized to /uas1), FLY_PATTERN_SKIP_PARAM,
     FLY_PATTERN_PARAM_WAIT_SEC (default 25),
     FLY_PATTERN_MAVROS_WAIT_SEC (default 90; overrides legacy FLY_PATTERN_UAS1_WAIT_SEC if set),
     FLY_PATTERN_LEGACY_MAVROS=1 to allow fallback to /mavros after timeout (not recommended),
     FLY_PATTERN_RECORD_BAG=0 to disable rosbag recording,
     FLY_PATTERN_BAG_DIR (default /home/uas/rosbags),
     FLY_PATTERN_BAG_TOPICS (comma-separated topics; default records all ROS 2 topics).
"""
from __future__ import annotations

from datetime import datetime
import argparse
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, ParamSet, ParamSetV2, SetMode
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import Image

# Integer params (SITL / noisy GPS / yaw preflight relaxation)
_SITL_INT_PARAMS: dict[str, int] = {
    'COM_ARM_WO_GPS': 1,
    'COM_ARM_SWISBTN': 1,
    'COM_RC_IN_MODE': 1,  # 1 = stick input disabled in PX4 v1.15
    'COM_RCL_EXCEPT': 4,  # ignore RC loss while in Offboard
    'NAV_RCL_ACT': 0,
    'NAV_DLL_ACT': 0,
    'COM_ARM_MAG_STR': 0,
    'EKF2_MAG_CHECK': 0,
    'EKF2_REQ_NSATS': 0,
    'CBRK_SUPPLY_CHK': 894281,
    'CBRK_IO_SAFETY': 22027,
    'CBRK_FLIGHTTERM': 121212,
}
# Float params — COM_ARM_EKF_* are maximum allowed EKF innovation ratios.
_SITL_FLOAT_PARAMS: dict[str, float] = {
    'COM_ARM_EKF_YAW': 1.0,
    'COM_ARM_EKF_POS': 1.0,
    'COM_ARM_EKF_VEL': 1.0,
    'COM_ARM_EKF_HGT': 1.0,
    'EKF2_REQ_GPS_H': 0.0,
    'COM_ARM_IMU_ACC': 30.0,
    'COM_ARM_IMU_GYR': 30.0,
}


_DEFAULT_MAVROS_NS = '/uas1'


_DEFAULT_SENSOR_TOPICS = [
    '/clock',
    # MAVROS state / estimator outputs
    '{base}/state',
    '{base}/extended_state',
    '{base}/sys_status',
    '{base}/estimator_status',
    '{base}/altitude',
    '{base}/vfr_hud',
    # IMU / pressure / temperature / magnetometer
    '{base}/imu/data',
    '{base}/imu/data_raw',
    '{base}/imu/mag',
    '{base}/imu/static_pressure',
    '{base}/imu/diff_pressure',
    '{base}/imu/temperature_baro',
    '{base}/imu/temperature_imu',
    # Position / velocity / odometry
    '{base}/local_position/pose',
    '{base}/local_position/pose_cov',
    '{base}/local_position/odom',
    '{base}/local_position/velocity_local',
    '{base}/local_position/velocity_body',
    '{base}/local_position/accel',
    '{base}/global_position/global',
    '{base}/global_position/local',
    '{base}/global_position/raw/fix',
    '{base}/global_position/raw/gps_vel',
    '{base}/global_position/raw/satellites',
    '{base}/global_position/rel_alt',
    # Simulation state and command/feedback topics useful for replay/debugging
    '{base}/sim_state/attitude',
    '{base}/sim_state/global_position',
    '{base}/sim_state/velocity_local',
    '{base}/sim_state/velocity_body',
    '{base}/sim_state/acceleration',
    '{base}/setpoint_position/local',
    '{base}/timesync_status',
    '/diagnostics',
    '/tf',
    '/tf_static',
    # Canonical camera topics (bridged from Gazebo).  Do not record the raw
    # Gazebo aliases as well: they are model-dependent and often have zero
    # samples, which makes post-flight topic selection ambiguous.
    '/camera/color/image_raw',
    '/camera/color/camera_info',
]


class RosbagRecorder:
    def __init__(self, node: Node, base: str) -> None:
        self._node = node
        self._base = base.rstrip('/')
        self._process: subprocess.Popen[str] | None = None
        self._log_file = None
        self._log_path: Path | None = None
        self._bag_marker: Path | None = None
        self.output_dir: Path | None = None

    @staticmethod
    def _enabled() -> bool:
        return os.environ.get('FLY_PATTERN_RECORD_BAG', '1').lower() not in ('0', 'false', 'no')

    def _topics(self) -> list[str]:
        explicit = os.environ.get('FLY_PATTERN_BAG_TOPICS', '').strip()
        if explicit:
            return [topic.strip() for topic in explicit.split(',') if topic.strip()]
        topics = [topic.format(base=self._base) for topic in _DEFAULT_SENSOR_TOPICS]
        record_depth = os.environ.get('FLY_PATTERN_RECORD_DEPTH', '').lower() in ('1', 'true', 'yes')
        require_depth = os.environ.get('FLY_PATTERN_REQUIRE_DEPTH', '').lower() in ('1', 'true', 'yes')
        if record_depth or require_depth:
            topics.append('/camera/depth/image_raw')
            topics.append('/points')
        return topics

    def start(self) -> None:
        if not self._enabled():
            self._node.get_logger().info('Sensor rosbag recording disabled (FLY_PATTERN_RECORD_BAG=0)')
            return

        root = Path(os.environ.get('FLY_PATTERN_BAG_DIR', '/home/uas/rosbags')).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self._bag_marker = root / '.active_bag'
        self._bag_marker.unlink(missing_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = root / f'fly_pattern_{stamp}'

        topics = self._topics()
        ros2_executable = shutil.which('ros2') or '/opt/ros/humble/bin/ros2'
        cmd = [ros2_executable, 'bag', 'record', '-o', str(self.output_dir), *topics]
        self._node.get_logger().info(
            f'Recording {len(topics)} explicit camera & telemetry topics to {self.output_dir}'
        )

        try:
            # rosbag2 owns the output directory and may remove files already
            # present there while opening storage. Keep the diagnostic log next
            # to the bag, not inside it.
            self._log_path = Path(f'{self.output_dir}.record.log')
            self._log_file = self._log_path.open(
                'w', encoding='utf-8'
            )
            env = dict(os.environ)
            env['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/uas/fastdds_udp.xml'
            self._process = subprocess.Popen(
                cmd,
                env=env,
                stdout=self._log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            time.sleep(1.0)
            if self._process.poll() is not None:
                self._log_file.flush()
                details = self._log_path.read_text(encoding='utf-8', errors='replace')
                raise RuntimeError(
                    f'rosbag recorder exited with code {self._process.returncode}: {details[-2000:]}'
                )
        except FileNotFoundError as exc:
            self._node.get_logger().error(f'Unable to execute rosbag recorder {cmd[0]}: {exc}')
            if self._log_file is not None:
                self._log_file.close()
                self._log_file = None
            self._process = None
            raise RuntimeError('rosbag recording could not be started') from exc
        except Exception as exc:
            self._node.get_logger().error(f'Unable to start rosbag recorder: {exc}')
            if self._log_file is not None:
                self._log_file.close()
                self._log_file = None
            self._process = None
            raise

    def stop(self) -> None:
        if self._process is None:
            return

        if self._process.poll() is None:
            self._node.get_logger().info('Stopping sensor rosbag recording...')
            os.killpg(self._process.pid, signal.SIGINT)
            try:
                self._process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._node.get_logger().warn('rosbag did not stop after SIGINT; terminating')
                os.killpg(self._process.pid, signal.SIGTERM)
                self._process.wait(timeout=5.0)

        if self.output_dir is not None:
            self._node.get_logger().info(f'Sensor rosbag saved: {self.output_dir}')
            if self._bag_marker is not None:
                # Trailing newline is required: shell consumers use `read`, which
                # returns a nonzero (EOF) exit code on a final line without one,
                # killing `set -e` scripts before post-flight analysis runs
                self._bag_marker.write_text(str(self.output_dir) + '\n', encoding='utf-8')
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
        self._process = None


def _normalize_mavros_ns(value: str) -> str:
    """Use the ROS namespace that contains mavros topics/services, not the node name itself."""
    ns = value.strip().rstrip('/')
    if ns.endswith('/mavros') and ns != '/mavros':
        ns = ns[: -len('/mavros')]
    return ns


def _pick_mavros_base(node: Node) -> str:
    """Resolve MAVROS namespace; default /uas1 (matches mavros_node --ros-args -r __ns:=/uas1)."""
    explicit = os.environ.get('MAVROS_NS', '').strip()
    base = _normalize_mavros_ns(explicit) if explicit else _DEFAULT_MAVROS_NS
    if explicit:
        node.get_logger().info(f'MAVROS_NS={explicit} -> using namespace {base}')
    else:
        node.get_logger().info(f'MAVROS namespace default {_DEFAULT_MAVROS_NS} (set MAVROS_NS to override)')

    wait = os.environ.get('FLY_PATTERN_MAVROS_WAIT_SEC', '').strip()
    if wait:
        timeout = float(wait)
    else:
        timeout = float(os.environ.get('FLY_PATTERN_UAS1_WAIT_SEC', '90'))

    arm = node.create_client(CommandBool, f'{base}/cmd/arming')
    if arm.wait_for_service(timeout_sec=timeout):
        node.get_logger().info(f'MAVROS cmd/arming ready at {base}')
        return base

    if (
        base == _DEFAULT_MAVROS_NS
        and os.environ.get('FLY_PATTERN_LEGACY_MAVROS', '').lower() in ('1', 'true', 'yes')
    ):
        node.get_logger().warn('FLY_PATTERN_LEGACY_MAVROS: trying /mavros/cmd/arming')
        legacy = '/mavros'
        arm_l = node.create_client(CommandBool, f'{legacy}/cmd/arming')
        if arm_l.wait_for_service(timeout_sec=15.0):
            return legacy

    node.get_logger().error(
        f'Timed out waiting for {base}/cmd/arming ({timeout}s). '
        'Start MAVROS with: ros2 run mavros mavros_node --ros-args -r __ns:=/uas1 -p fcu_url:=udp://:14540@127.0.0.1:14580'
    )
    return base


def _pick_param_client(node: Node, base: str) -> Optional[tuple[Any, str]]:
    """Try param/set in the resolved MAVROS namespace, plus old /mavros-node compatibility paths."""
    # PX4 boot parameters are installed by bootstrap_px4.sh. Avoid the
    # fragile MAVROS parameter round-trip by default; opt in explicitly when
    # testing a different PX4 parameter set.
    skip = os.environ.get('FLY_PATTERN_SKIP_PARAM', '1').lower() in ('1', 'true', 'yes')
    if skip:
        node.get_logger().info('Skipping MAVROS param push (FLY_PATTERN_SKIP_PARAM=1)')
        return None

    timeout = float(os.environ.get('FLY_PATTERN_PARAM_WAIT_SEC', '25'))
    b = base.rstrip('/')
    candidates: list[tuple[str, Any, str]] = [
        (f'{b}/param/set', ParamSetV2, 'v2'),
        (f'{b}/mavros/param/set', ParamSet, 'legacy'),
    ]
    for cand, srv_type, kind in candidates:
        cli = node.create_client(srv_type, cand)
        if cli.wait_for_service(timeout_sec=timeout):
            node.get_logger().info(f'Using param service {cand} ({kind})')
            return cli, kind
    node.get_logger().warn(
        f'param/set not available within {timeout}s — proceeding with default SITL params.'
    )
    return None


# ros_gz_bridge publishes camera images with RELIABLE QoS. Subscribing with
# BEST_EFFORT (qos_profile_sensor_data) causes a QoS mismatch — zero frames
# are delivered and fly_pattern.py hangs forever. Use a matching RELIABLE QoS.
_CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class SimpleFlyer(Node):
    def __init__(self) -> None:
        super().__init__('simple_flyer')
        self._base = _pick_mavros_base(self)

        self.pub = self.create_publisher(PoseStamped, f'{self._base}/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, f'{self._base}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'{self._base}/set_mode')
        self.create_subscription(State, f'{self._base}/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, f'{self._base}/local_position/pose', self._pos_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw', self._rgb_cb, _CAMERA_QOS)
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, _CAMERA_QOS)
        self.current_state: State | None = None
        self.has_local_pos = False
        self.has_rgb_frame = False
        self.has_depth_frame = False
        self.rgb_frame_count = 0
        self.depth_frame_count = 0

        # Continuous background setpoint streamer (20 Hz)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.set_target(0.0, 0.0, 1.0, 1.5707963)
        self._sp_timer = self.create_timer(0.05, self._stream_timer_cb)

        for svc in (self.arming_client, self.mode_client):
            while not svc.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {svc.srv_name}...')

        self._param_client = _pick_param_client(self, self._base)
        self._recorder = RosbagRecorder(self, self._base)

    def _state_cb(self, msg: State) -> None:
        self.current_state = msg

    def _pos_cb(self, msg: PoseStamped) -> None:
        self.has_local_pos = True

    def _rgb_cb(self, msg: Image) -> None:
        self.rgb_frame_count += 1
        if not self.has_rgb_frame:
            self.get_logger().info(f'RGB frame received: {msg.width}x{msg.height} {msg.encoding}')
        self.has_rgb_frame = True

    def _depth_cb(self, msg: Image) -> None:
        self.depth_frame_count += 1
        if not self.has_depth_frame:
            self.get_logger().info(f'Depth frame received: {msg.width}x{msg.height} {msg.encoding}')
        self.has_depth_frame = True

    def wait_for_position_estimate(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info('Waiting for EKF2 position estimate on local_position/pose...')
        t_end = time.time() + timeout_sec
        while time.time() < t_end:
            if self.has_local_pos:
                self.get_logger().info('Position estimate active and ready!')
                return True
            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.5)
        self.get_logger().warn('Position estimate wait timed out.')
        return False

    def _stream_timer_cb(self) -> None:
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.target_pose)

    def set_target(self, x: float, y: float, z: float, yaw_rad: float = 1.5707963) -> None:
        self.target_pose.pose.position.x = float(x)
        self.target_pose.pose.position.y = float(y)
        self.target_pose.pose.position.z = float(z)
        half = float(yaw_rad) * 0.5
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = math.sin(half)
        self.target_pose.pose.orientation.w = math.cos(half)

    def set_param(self, param_id: str, value: float | int, use_float: bool) -> bool:
        if self._param_client is None:
            return False
        cli, kind = self._param_client
        if kind == 'v2':
            req = ParamSetV2.Request()
            req.force_set = True
            req.param_id = param_id
            if use_float:
                req.value.type = ParameterType.PARAMETER_DOUBLE
                req.value.double_value = float(value)
            else:
                req.value.type = ParameterType.PARAMETER_INTEGER
                req.value.integer_value = int(value)
        else:
            req = ParamSet.Request()
            req.param_id = param_id
            if use_float:
                req.value.real = float(value)
            else:
                req.value.integer = int(value)
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().warn(f'param set timed out: {param_id}')
            return False
        res = future.result()
        return res is not None and res.success

    def push_sitl_params(self) -> None:
        if self._param_client is not None:
            self.get_logger().info('Pushing SITL / GNSS-denied friendly params via MAVROS...')
            for name, val in _SITL_INT_PARAMS.items():
                if self.set_param(name, val, use_float=False):
                    self.get_logger().info(f'Set param {name}={val}')
                else:
                    self.get_logger().warn(f'Failed to set param {name}')
            for name, val in _SITL_FLOAT_PARAMS.items():
                if self.set_param(name, val, use_float=True):
                    self.get_logger().info(f'Set param {name}={val}')
                else:
                    self.get_logger().warn(f'Failed to set param {name}')
        self.get_logger().info('Waiting for estimator / preflight to settle...')
        time.sleep(3.0)

    def arm_with_retries(self, attempts: int = 30, delay: float = 1.0) -> bool:
        for i in range(attempts):
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            res = future.result()
            if res is not None and res.success:
                self.get_logger().info('Armed successfully via MAVROS!')
                return True
            if self.current_state and self.current_state.armed:
                self.get_logger().info('Armed confirmed via State topic!')
                return True

            # Robust SITL fallback: trigger commander arm directly
            try:
                subprocess.run(
                    ['/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander', 'arm', '-f'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2.0,
                )
            except Exception:
                pass

            if self.current_state and self.current_state.armed:
                self.get_logger().info('Armed successfully via px4-commander fallback!')
                return True
            self.get_logger().warn(f'Arm attempt {i + 1}/{attempts} failed (waiting for EKF2/preflight) — retrying...')
            time.sleep(delay)
        return False

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()
        return res is not None and res.mode_sent

    def set_mode_with_retries(self, mode: str, attempts: int = 5) -> bool:
        for i in range(attempts):
            if self.set_mode(mode):
                time.sleep(0.5)
                if self.current_state is None or self.current_state.mode == mode:
                    return True
            self.get_logger().warn(f'{mode} attempt {i + 1}/{attempts} failed — retrying...')
            time.sleep(1.0)
        return self.current_state is not None and self.current_state.mode == mode

    def wait_for_camera_topics(self, timeout_sec: float = 30.0) -> bool:
        require_depth = os.environ.get('FLY_PATTERN_REQUIRE_DEPTH', '0').lower() in (
            '1', 'true', 'yes'
        )
        self.get_logger().info(
            'Waiting for live camera frames before recording & takeoff '
            f'(depth required={require_depth})...'
        )
        t_end = time.time() + timeout_sec
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.rgb_frame_count >= 2 and (not require_depth or self.depth_frame_count >= 2):
                self.get_logger().info('Required camera streams are publishing live frames.')
                return True
        self.get_logger().error(
            'Camera readiness failed: '
            f'rgb={self.has_rgb_frame}, depth={self.has_depth_frame}, '
            f'depth_required={require_depth}'
        )
        return False

    def fly_to(self, x: float, y: float, z: float, duration: float, yaw_rad: float = 1.5707963, desc: str = '') -> None:
        if desc:
            self.get_logger().info(f'Fly to x={x:.1f} y={y:.1f} z={z:.1f} ({duration}s) — {desc}')
        else:
            self.get_logger().info(f'Fly to x={x:.1f} y={y:.1f} z={z:.1f} ({duration}s)')
        self.set_target(x, y, z, yaw_rad)
        t_end = time.time() + duration
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def run_pattern(self) -> None:
        self.push_sitl_params()

        # Wait for EKF2 position estimate & camera topics before recording bag
        self.wait_for_position_estimate(timeout_sec=30.0)
        if not self.wait_for_camera_topics(timeout_sec=90.0):
            raise RuntimeError('Required camera frames are not publishing; refusing to fly')

        self._recorder.start()
        try:
            self.get_logger().info('Streaming initial setpoints (OFFBOARD precondition)...')
            self.set_target(0.0, 0.0, 1.0, 1.5707963)
            t_end = time.time() + 5.0
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)

            if not self.set_mode_with_retries('OFFBOARD'):
                self.get_logger().error('Failed to enter OFFBOARD')
                return
            self.get_logger().info('OFFBOARD requested')

            if not self.arm_with_retries():
                self.get_logger().error('Arming failed after retries.')
                return
            self.get_logger().info('Armed')

            if self.current_state and self.current_state.mode != 'OFFBOARD':
                self.get_logger().warn(f'FCU mode is {self.current_state.mode!r}, re-requesting OFFBOARD')
                self.set_mode('OFFBOARD')

            # Inspection facade pattern: Wall is at Y=5.0m. Drone hovers at Y=2.5m facing the wall (yaw=1.57 rad).
            # Markers at X=-1.2m (crack) and X=+1.8m (erosion) are directly in camera view.
            inspection_waypoints = [
                (0.0, 2.5, 2.0, 8.0, "Takeoff and position 2.5m facing earthen wall"),
                (-3.5, 2.5, 2.0, 10.0, "Move to left boundary of wall facade"),
                (3.5, 2.5, 2.0, 16.0, "Scan across wall facade (sweeping past crack and erosion markers)"),
                (3.5, 2.5, 2.8, 6.0, "Ascend to upper inspection altitude (2.8m)"),
                (-3.5, 2.5, 2.8, 16.0, "Scan back across upper facade"),
                (0.0, 1.5, 1.5, 8.0, "Return towards landing zone"),
                (0.0, 0.0, 0.3, 6.0, "Descend over home point"),
            ]
            for x, y, z, duration, desc in inspection_waypoints:
                self.fly_to(x, y, z, duration, yaw_rad=1.5707963, desc=desc)

            self.get_logger().info('AUTO.LAND')
            # Destroy timer so setpoint publisher stops and lets PX4 land
            self._sp_timer.cancel()
            self.set_mode('AUTO.LAND')
            time.sleep(10)
            self.get_logger().info('Pattern complete')
        finally:
            self._recorder.stop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    flyer = SimpleFlyer()
    flyer.run_pattern()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
