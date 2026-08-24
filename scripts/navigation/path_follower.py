#!/usr/bin/env python3
"""
path_follower.py  (Navigation Phase 1 - Coverage + Revisit support)

Changes vs original:
  1. Subscribes to /planner/coverage_path in addition to /planned_path
     (coverage planner pushes on /planner/coverage_path; A* pushes on /planned_path).
  2. Forwards full PoseStamped (position + orientation) to MAVROS so yaw is
     preserved — previously only position was forwarded, causing yaw=0 (facing away
     from wall).
  3. Adds a sequential revisit-leg queue: revisit waypoints accumulate in a list and
     execute one-by-one after the primary coverage pass completes. This prevents the
     "last path wins" bug where all but the final revisit leg were discarded.
  4. Exposes /path_follower/status (String) for integration-test assertions.

MAVROS setpoint publisher is the sole owner of /uas1/setpoint_position/local.
fly_pattern.py must NOT run simultaneously with this node.
"""

import math
import threading
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import numpy as np


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ---- Internal state --------------------------------------------------
        # Each element is a PoseStamped (position + orientation).
        self.path: list[PoseStamped] = []
        # Revisit legs queued by revisit_waypoint_generator via /planned_path.
        # Elements are nav_msgs/Path messages; consumed sequentially after primary pass.
        self._revisit_queue: list[list[PoseStamped]] = []
        self._primary_done = False

        self.state = State()
        self.pose: PoseStamped | None = None
        self.battery = None
        self.low_battery = False
        self.taking_off = True
        self._last_req_time = 0.0
        # Hover hold point: latched once when entering hover, NOT continuously
        # re-sampled. Streaming the live estimate back as a setpoint lets EKF
        # drift drag the vehicle across the world (observed 1.3 km runaway).
        self._hover_anchor: PoseStamped | None = None

        # Position tolerance (m) to declare a waypoint reached
        self.declare_parameter('position_tolerance_m', 0.6)
        self.tol = self.get_parameter('position_tolerance_m').value

        # ---- QoS -------------------------------------------------------------
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Publishers ------------------------------------------------------
        self.sp_pub = self.create_publisher(
            PoseStamped, '/uas1/setpoint_position/local', 10
        )
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(String, '/path_follower/status', status_qos)
        self.current_status = 'STARTING'
        self._publish_status('STARTING')

        # ---- Subscriptions ---------------------------------------------------
        # MAVROS state
        self.create_subscription(State, '/uas1/state', self._state_cb, 10)
        # MAVROS local pose
        self.create_subscription(PoseStamped, '/uas1/local_position/pose',
                                 self._pose_cb, qos_be)
        # Battery
        self.create_subscription(BatteryState, '/uas1/battery',
                                 self._battery_cb, qos_be)
        # A* planned path (obstacle routing / manual goals / revisit legs)
        self.create_subscription(Path, '/planned_path', self._planned_path_cb, 10)
        # Coverage planner path (primary lawnmower pass).
        # TRANSIENT_LOCAL must match the publisher's latched QoS — otherwise ROS2
        # silently rejects the connection and the subscriber never receives anything.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, '/planner/coverage_path',
                                 self._coverage_path_cb, latched_qos)

        # ---- Service clients -------------------------------------------------
        self.mode_cli = self.create_client(SetMode, '/uas1/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/uas1/cmd/arming')

        # ---- 20 Hz control loop ---------------------------------------------
        self.create_timer(0.05, self._timer_cb)

        # ---- Background startup worker ---------------------------------------
        threading.Thread(target=self._startup_worker, daemon=True).start()

        self.get_logger().info('PathFollower initialised (yaw-aware, coverage + revisit queue).')

    # ---- Callbacks -----------------------------------------------------------

    def _state_cb(self, msg: State):
        if msg.mode != self.state.mode:
            self.get_logger().info(f'FCU mode: {msg.mode}')
        if msg.armed != self.state.armed:
            self.get_logger().info(f'Armed: {msg.armed}')
        self.state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def _battery_cb(self, msg: BatteryState):
        self.battery = msg
        # In SITL, simulated battery is uninitialized (0.0). Only trigger on real low battery.
        if 0.05 < msg.percentage < 0.20 and not self.low_battery:
            self.get_logger().error(
                f'LOW BATTERY {msg.percentage*100:.0f}%! RTH...'
            )
            self.low_battery = True
            self.path = []
            self._revisit_queue.clear()

    def _coverage_path_cb(self, msg: Path):
        """Primary lawnmower coverage path. Replaces current path immediately."""
        if self.low_battery:
            return
        if not msg.poses:
            return
        self._primary_done = False
        self._hover_anchor = None
        self.path = list(msg.poses)
        self.get_logger().info(
            f'[PathFollower] Coverage path received: {len(self.path)} waypoints.'
        )
        self._publish_status('COVERAGE_ACTIVE')

    def _planned_path_cb(self, msg: Path):
        """
        A*-planned path (obstacle routing or revisit leg).
        """
        if self.low_battery:
            return
        if not msg.poses:
            return

        if not self._primary_done and self.path:
            # Coverage still in progress — enqueue revisit for later
            self._revisit_queue.append(list(msg.poses))
            self.get_logger().info(
                f'[PathFollower] Revisit leg queued '
                f'({len(self._revisit_queue)} in queue).'
            )
        else:
            # No active coverage path — execute now
            self._hover_anchor = None
            self.path = list(msg.poses)
            self.get_logger().info(
                f'[PathFollower] Planned path received: {len(self.path)} waypoints.'
            )

    # ---- Background startup worker -------------------------------------------

    def _startup_worker(self):
        self.get_logger().info('[Startup] Waiting for /uas1/cmd/arming service...')
        self.arm_cli.wait_for_service(timeout_sec=60.0)
        self.mode_cli.wait_for_service(timeout_sec=60.0)

        # Wait for EKF2 position estimate
        self.get_logger().info('[Startup] Waiting for EKF2 position estimate...')
        for _ in range(60):
            if self.pose is not None:
                break
            time.sleep(0.5)

        # Configure SITL params
        self.get_logger().info('[Startup] Configuring SITL parameters...')
        for param, val in [
            ('COM_ARM_WO_GPS', '1'),
            ('COM_RC_IN_MODE', '1'),
            ('CBRK_IO_SAFETY', '22027'),
            ('CBRK_SUPPLY_CHK', '894281'),
            ('COM_LOW_BAT_ACT', '0'),
            ('BAT_LOW_THR', '0.0'),
            ('BAT_CRIT_THR', '0.0'),
            ('BAT_EMERGEN_THR', '0.0'),
            ('COM_DISARM_PRFLT', '0'),
            ('COM_RCL_EXCEPT', '4'),
            ('NAV_RCL_ACT', '0'),
            ('NAV_DLL_ACT', '0'),
        ]:
            try:
                subprocess.run(
                    ['/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4-param', 'set', param, val],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0
                )
            except Exception:
                pass

        # Stream setpoints for 3 seconds before requesting mode switch
        self.get_logger().info('[Startup] Streaming initial setpoints (OFFBOARD precondition)...')
        time.sleep(3.0)

        # Switch to OFFBOARD mode
        self.get_logger().info('[Startup] Requesting OFFBOARD mode...')
        for _ in range(10):
            if self.state.mode == 'OFFBOARD':
                break
            req = SetMode.Request(custom_mode='OFFBOARD')
            self.mode_cli.call_async(req)
            time.sleep(0.5)

        # Arm the vehicle
        self.get_logger().info('[Startup] Arming vehicle...')
        for _ in range(15):
            if self.state.armed:
                break
            req = CommandBool.Request(value=True)
            self.arm_cli.call_async(req)
            time.sleep(0.4)
            if not self.state.armed:
                try:
                    subprocess.run(
                        ['/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander', 'arm', '-f'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0
                    )
                except Exception:
                    pass
            time.sleep(0.4)

        # Ensure OFFBOARD mode is active after arming
        for _ in range(10):
            if self.state.mode == 'OFFBOARD':
                break
            try:
                subprocess.run(
                    ['/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander', 'mode', 'offboard'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0
                )
            except Exception:
                pass
            req = SetMode.Request(custom_mode='OFFBOARD')
            self.mode_cli.call_async(req)
            time.sleep(0.3)

        if self.state.armed:
            self.get_logger().info('>>> Drone ARMED and in OFFBOARD mode! Lifting off to 1.5m...')

        # Climb to takeoff altitude
        z_start = self.pose.pose.position.z if self.pose else 0.0
        t_start = time.time()
        while self.taking_off and (time.time() - t_start) < 8.0:
            if self.pose and (self.pose.pose.position.z - z_start) > 0.5:
                self.get_logger().info(f'>>> Takeoff altitude reached (Z={self.pose.pose.position.z:.2f}m). Starting inspection sweep!')
                self.taking_off = False
                break
            time.sleep(0.2)
        self.taking_off = False
        self.get_logger().info('>>> Proceeding to coverage inspection waypoints!')

    # ---- 20 Hz timer ---------------------------------------------------------

    def _timer_cb(self):
        sp = PoseStamped()
        sp.header.frame_id = 'map'
        sp.header.stamp = self.get_clock().now().to_msg()
        # Default orientation: facing +Y earthen wall (yaw = pi/2)
        sp.pose.orientation.z = math.sin(math.pi / 4.0)
        sp.pose.orientation.w = math.cos(math.pi / 4.0)

        # Periodically publish status
        status_msg = String()
        status_msg.data = self.current_status
        self.status_pub.publish(status_msg)

        # Enforce OFFBOARD mode during active flight
        if not self.low_battery and self.state.armed and self.state.mode != 'OFFBOARD':
            self._call_set_mode('OFFBOARD')

        if self.low_battery:
            # Emergency RTH
            sp.pose.position.x = 0.0
            sp.pose.position.y = 0.0
            sp.pose.position.z = 1.5
            if self.pose and self._distance_to(0.0, 0.0, 1.5) < 0.5:
                self.get_logger().info('Home reached — landing.')
                self._call_set_mode('AUTO.LAND')
        elif self.taking_off:
            # Climb and position in front of wall at 2.0 m while streaming setpoints
            sp.pose.position.x = 0.0
            sp.pose.position.y = 2.25
            sp.pose.position.z = 2.0
        elif self.path:
            # ---- Waypoint tracking (preserves orientation for yaw) -----------
            target: PoseStamped = self.path[0]
            sp.pose = target.pose        # copy position AND orientation

            if self.pose:
                tx = target.pose.position.x
                ty = target.pose.position.y
                tz = target.pose.position.z
                dist = self._distance_to(tx, ty, tz)
                cx = self.pose.pose.position.x
                cy = self.pose.pose.position.y
                cz = self.pose.pose.position.z

                self.get_logger().info(
                    f'Flying to wp ({tx:.2f}, {ty:.2f}, {tz:.2f}) | Current: ({cx:.2f}, {cy:.2f}, {cz:.2f}) | Dist: {dist:.2f}m | Left: {len(self.path)}',
                    throttle_duration_sec=2.0
                )

                if dist < self.tol:
                    self.path.pop(0)
                    remaining = len(self.path)
                    self.get_logger().info(
                        f'>>> Waypoint reached! {remaining} waypoints remaining.'
                    )
                    if remaining == 0:
                        self._on_path_complete()
        elif self.pose:
            # Hover at the latched anchor (see __init__ note on feedback loops)
            if self._hover_anchor is None:
                self._hover_anchor = PoseStamped()
                self._hover_anchor.header.frame_id = 'odom'
                self._hover_anchor.pose = self.pose.pose
                self._hover_anchor.pose.position.z = max(
                    self.pose.pose.position.z, 1.5)
                self.get_logger().info(
                    'Hover anchor latched at (%.2f, %.2f, %.2f).'
                    % (self._hover_anchor.pose.position.x,
                       self._hover_anchor.pose.position.y,
                       self._hover_anchor.pose.position.z))
            sp.pose = self._hover_anchor.pose
        else:
            return  # No pose yet; skip publish

        self.sp_pub.publish(sp)

    # ---- Path-complete handler -----------------------------------------------

    def _on_path_complete(self):
        """Called each time the current path list empties."""
        self._hover_anchor = None  # next hover re-latches at the arrival point
        if not self._primary_done:
            # Primary coverage pass just finished
            self._primary_done = True
            self.get_logger().info(
                '[PathFollower] Primary coverage pass complete. '
                f'{len(self._revisit_queue)} revisit leg(s) queued.'
            )
            self._publish_status('COVERAGE_DONE')
            self._advance_revisit_queue()
        else:
            # A revisit leg finished
            self.get_logger().info('[PathFollower] Revisit leg complete.')
            self._publish_status('REVISIT_LEG_DONE')
            self._advance_revisit_queue()

    def _advance_revisit_queue(self):
        """Pop and execute the next revisit leg (if any)."""
        if self._revisit_queue:
            next_leg = self._revisit_queue.pop(0)
            self.path = next_leg
            self.get_logger().info(
                f'[PathFollower] Starting revisit leg '
                f'({len(next_leg)} pts, '
                f'{len(self._revisit_queue)} still queued).'
            )
            self._publish_status('REVISIT_ACTIVE')
        else:
            self.get_logger().info('[PathFollower] All legs complete. Hovering.')
            self._publish_status('IDLE')

    # ---- Helpers -------------------------------------------------------------

    def _distance_to(self, x: float, y: float, z: float) -> float:
        if self.pose is None:
            return float('inf')
        p = self.pose.pose.position
        return math.sqrt((p.x - x)**2 + (p.y - y)**2 + (p.z - z)**2)

    def _call_set_mode(self, mode: str):
        req = SetMode.Request(custom_mode=mode)
        self.mode_cli.call_async(req)
        if mode == 'OFFBOARD':
            try:
                subprocess.Popen(
                    ['ros2', 'service', 'call', '/uas1/set_mode', 'mavros_msgs/srv/SetMode', '{custom_mode: "OFFBOARD"}'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            except Exception:
                pass

    def _publish_status(self, status: str):
        self.current_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = PathFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
