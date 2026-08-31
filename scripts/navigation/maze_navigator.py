#!/usr/bin/env python3
"""
maze_navigator.py — Autonomous maze navigation via MAVROS + A* planner.

State machine (in order):
  WAITING         wait for MAVROS connection + first EKF2 pose + EKF2 convergence
  SETTING_OFFBOARD stream setpoints at current position; request OFFBOARD + ARM
  ARMING          explicitly arm the drone
  TAKEOFF         climb to safe altitude (holds current x,y — does not drift into walls!)
  FLYING/HOVERING follow A* waypoints or hold frozen position between replans
  SUCCESS         goal reached; exit 0
  TIMEOUT         elapsed > --timeout; exit 1
  FAULT           envelope violation / crash; exit 2

PX4 OFFBOARD rules obeyed:
  - Setpoints published at 20 Hz at ALL times (required for OFFBOARD)
  - OFFBOARD requested only after setpoints streaming AND EKF2 converged
  - Takeoff holds current x,y — avoids driving drone into nearby maze wall
  - Hover hold uses FROZEN position — live pose gives near-zero thrust!

Argument parsing:
  Uses rclpy.utilities.remove_ros_args() to strip --ros-args from sys.argv
  BEFORE calling argparse. This fixes the bug where argparse ignores --timeout etc.
  when they appear after the '--' ROS separator.
"""
import argparse, math, os, sys, time, csv

ros_site = "/opt/ros/humble/lib/python3.10/site-packages"
if ros_site not in sys.path and os.path.exists(ros_site):
    sys.path.append(ros_site)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

# ---------------------------------------------------------------------------
GOAL_TOL_M        = 0.2    # goal success radius (m)
WP_TOL_M          = 0.3    # advance to next waypoint (m)
TAKEOFF_Z_M       = 1.8    # climb target (m)
TAKEOFF_DONE_Z_M  = 1.5    # altitude at which takeoff is considered done (m)
HZ                = 20     # control loop rate
ENVELOPE_XY       = 20.0   # max |x| or |y| in local frame before fault (m) — increased from 12 to allow overshoot and recovery; maze is ±8, goal 7, so 20 gives margin
ENVELOPE_Z_LOW    = -2.0   # min z before fault (m) — relaxed to prevent premature aborts on rough spawn
ENVELOPE_Z_HIGH   = 10.0   # max z before fault (m)
GOAL_RESEND_S     = 3.0    # re-publish goal to planner this often (s)
EKF_STABLE_WIN    = 3.0    # how long position must be stable before arming (s) — 3s is enough for EKF, was 5s too strict and never passed due to slow z drift
EKF_STABLE_TOL    = 0.5    # max position jump (m) in EKF_STABLE_WIN to count as stable — 0.5 allows for baro drift, was 0.3 too tight


class MazeNavigator(Node):
    def __init__(self, goal_x, goal_y, goal_z, timeout, csv_path):
        super().__init__('maze_navigator')
        self.gx, self.gy, self.gz = float(goal_x), float(goal_y), float(goal_z)
        self.timeout = float(timeout)

        self.pose           = None   # latest EKF2 pose (geometry_msgs/Pose)
        self.state          = State()
        self.path           = []     # current A* path waypoints
        self.hover_pose     = None   # frozen hold position
        self.t0             = None
        self.last_pos       = None
        self.last_goal_t    = 0.0
        # EKF2 convergence tracking
        self._ekf_window    = []     # (time, x, y, z) samples

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # State is RELIABLE/TRANSIENT_LOCAL latching; match it to avoid missing arm/mode updates
        from rclpy.qos import DurabilityPolicy, HistoryPolicy
        qos_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PoseStamped, '/uas1/local_position/pose',
                                 lambda m: self._on_pose(m), qos_be)
        self.create_subscription(State, '/uas1/state', self._state_cb, qos_state)
        self.create_subscription(Path, '/planned_path', self._path_cb, 10)

        self.sp_pub   = self.create_publisher(PoseStamped, '/uas1/setpoint_position/local', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/navigation/goal', 10)
        self.mode_cli = self.create_client(SetMode, '/uas1/set_mode')
        self.arm_cli  = self.create_client(CommandBool, '/uas1/cmd/arming')

        os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
        self._f = open(csv_path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(['t','x','y','z','dist','speed','mode','armed','path_n','status'])

        self.create_timer(1.0 / HZ, self._loop)
        self.get_logger().info(
            f'MazeNavigator ready. goal=({goal_x},{goal_y},{goal_z}) timeout={timeout}s')

    # ── callbacks ─────────────────────────────────────────────────────────
    def _on_pose(self, msg):
        self.pose = msg.pose
        p = msg.pose.position
        now = time.monotonic()
        self._ekf_window.append((now, p.x, p.y, p.z))
        # Keep only the last EKF_STABLE_WIN seconds, plus a small margin to ensure
        # the window duration can mathematically reach EKF_STABLE_WIN.
        cutoff = now - (EKF_STABLE_WIN + 0.5)
        self._ekf_window = [(t,x,y,z) for t,x,y,z in self._ekf_window if t >= cutoff]

    def _state_cb(self, msg):
        if msg.mode  != self.state.mode:  self.get_logger().info(f'Mode: {msg.mode}')
        if msg.armed != self.state.armed: self.get_logger().info(f'Armed: {msg.armed}')
        self.state = msg

    def _path_cb(self, msg):
        if msg.poses:
            self.path = list(msg.poses)
            if self.pose: self.hover_pose = self.pose
            self.get_logger().info(f'New path: {len(self.path)} waypoints')

    # ── helpers ────────────────────────────────────────────────────────────
    def _d(self, x, y, z):
        if self.pose is None: return float('inf')
        p = self.pose.position
        return math.sqrt((p.x-x)**2 + (p.y-y)**2 + (p.z-z)**2)

    def _sp(self, x, y, z, yaw=0.0):
        m = PoseStamped()
        # Navigation stack world frame is 'odom' (tf_bridge publishes odom->base_link
        # from MAVROS pose which claims frame_id='map' — we alias map as odom).
        # Keep setpoints in 'odom' to match octomap/planner world frame.
        m.header.frame_id    = 'odom'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(z)
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        self.sp_pub.publish(m)

    def _send_goal(self):
        m = PoseStamped()
        m.header.frame_id    = 'odom'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.pose.position.x    = self.gx
        m.pose.position.y    = self.gy
        m.pose.position.z    = self.gz
        m.pose.orientation.w = 1.0
        self.goal_pub.publish(m)
        self.last_goal_t = time.monotonic()
        self.get_logger().info(f'Goal sent: ({self.gx},{self.gy},{self.gz})')

    def _ekf_stable(self):
        """Return True if EKF2 position estimate has been stable for EKF_STABLE_WIN seconds."""
        # Copy to avoid race with _on_pose which reassigns the list
        w = list(self._ekf_window)
        if len(w) < 10:
            return False
        dur = w[-1][0] - w[0][0]
        if dur < EKF_STABLE_WIN:
            return False
        xs = [x for _,x,_,_ in w]
        ys = [y for _,_,y,_ in w]
        zs = [z for _,_,_,z in w]
        span_xy = max(max(xs)-min(xs), max(ys)-min(ys))
        span_z  = max(zs)-min(zs)
        span = max(span_xy, span_z)
        ok = span < EKF_STABLE_TOL
        if ok:
            self.get_logger().info(f'EKF2 converged (span={span:.3f}m xy={span_xy:.3f} z={span_z:.3f} over {EKF_STABLE_WIN}s)')
        return ok

    def _log(self, t, status):
        if self.pose is None: return
        p     = self.pose.position
        dist  = self._d(self.gx, self.gy, self.gz)
        speed = 0.0
        if self.last_pos:
            dx,dy,dz = p.x-self.last_pos[0], p.y-self.last_pos[1], p.z-self.last_pos[2]
            speed = math.sqrt(dx*dx+dy*dy+dz*dz) * HZ
        self.last_pos = (p.x, p.y, p.z)
        self._w.writerow([f'{t:.2f}', f'{p.x:.4f}', f'{p.y:.4f}', f'{p.z:.4f}',
                          f'{dist:.4f}', f'{speed:.4f}',
                          self.state.mode, self.state.armed, len(self.path), status])

    # ── main control loop (20 Hz) ──────────────────────────────────────────
    def _loop(self):
        if self.t0 is None: self.t0 = time.monotonic()
        t = time.monotonic() - self.t0

        # Envelope check (only once EKF has data)
        if self.pose and t > 5.0:
            p = self.pose.position
            if (abs(p.x) > ENVELOPE_XY or abs(p.y) > ENVELOPE_XY or
                    p.z < ENVELOPE_Z_LOW or p.z > ENVELOPE_Z_HIGH):
                self.get_logger().error(
                    f'Envelope violation: ({p.x:.1f},{p.y:.1f},{p.z:.1f})')
                self._log(t, 'FAULT')
                raise SystemExit(2)

        # Timeout
        if t > self.timeout:
            self.get_logger().error(f'Timeout after {t:.0f}s')
            self._log(t, 'TIMEOUT')
            raise SystemExit(1)

        # Wait for connection + first valid pose
        if not self.state.connected or self.pose is None:
            self._sp(0.0, 0.0, TAKEOFF_Z_M)
            self._log(t, 'WAITING')
            return

        px, py = self.pose.position.x, self.pose.position.y

        # Wait for EKF2 to converge before doing anything (prevents physics explosion)
        if not hasattr(self, 'ekf_ready'):
            self.ekf_ready = False
            self.takeoff_xy = None

        if not self.ekf_ready:
            is_stable = self._ekf_stable()
            if not is_stable:
                if t > 60.0:
                    self.get_logger().error(f'EKF failed to stabilize after {t:.1f}s, aborting mission.')
                    self._log(t, 'FAULT')
                    raise SystemExit(2)
                else:
                    self._sp(px, py, TAKEOFF_Z_M)   # stream setpoints but don't arm yet
                    # Debug: log EKF window stats every 2s (copy to avoid race)
                    if int(t*HZ) % (2*HZ) == 0:
                        w = list(self._ekf_window)
                        if len(w) >= 2:
                            span_xy = max(max(x for _,x,_,_ in w)-min(x for _,x,_,_ in w), max(y for _,_,y,_ in w)-min(y for _,_,y,_ in w))
                            span_z = max(z for _,_,_,z in w)-min(z for _,_,_,z in w)
                            dur = w[-1][0]-w[0][0]
                            self.get_logger().info(f'EKF_WAIT t={t:.1f} window={len(w)} dur={dur:.1f}s span_xy={span_xy:.3f} span_z={span_z:.3f} pose=({px:.2f},{py:.2f},{self.pose.position.z:.2f})')
                    self._log(t, 'EKF_WAIT')
                    return
            else:
                self.ekf_ready = True
                self.takeoff_xy = (px, py)
                q = self.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.takeoff_yaw = math.atan2(siny_cosp, cosy_cosp)
                self.get_logger().info(f'EKF converged at t={t:.1f}s, proceeding to flight.')

        tx, ty = self.takeoff_xy
        tyaw = self.takeoff_yaw if hasattr(self, 'takeoff_yaw') else 0.0

        # Request OFFBOARD + arm (hold current x,y during this phase)
        if self.state.mode != 'OFFBOARD':
            self.takeoff_xy = (px, py)
            self._sp(px, py, TAKEOFF_Z_M, tyaw)
            self.mode_cli.call_async(SetMode.Request(custom_mode='OFFBOARD'))
            if not self.state.armed:
                self.arm_cli.call_async(CommandBool.Request(value=True))
            self._log(t, 'SETTING_OFFBOARD')
            return

        if not self.state.armed:
            self.takeoff_xy = (px, py)
            self._sp(px, py, TAKEOFF_Z_M, tyaw)
            self.arm_cli.call_async(CommandBool.Request(value=True))
            self._log(t, 'ARMING')
            return

        # Takeoff — climb to safe altitude before navigating
        if not getattr(self, 'takeoff_done', False):
            if self.pose.position.z < TAKEOFF_DONE_Z_M:
                if self.pose.position.z < 0.5:
                    # Avoid ground friction/tilt lock by updating target to current
                    # position until we are clearly off the ground, then lock it.
                    self.takeoff_xy = (px, py)
                    self._sp(px, py, TAKEOFF_Z_M, tyaw)
                else:
                    self._sp(tx, ty, TAKEOFF_Z_M, tyaw)
                self._log(t, 'TAKEOFF')
                return
            else:
                self.takeoff_done = True

        # Periodically re-send goal to planner
        if time.monotonic() - self.last_goal_t > GOAL_RESEND_S:
            self._send_goal()

        # Success
        if self._d(self.gx, self.gy, self.gz) < GOAL_TOL_M:
            self.get_logger().info(f'SUCCESS! t={t:.1f}s dist={self._d(self.gx,self.gy,self.gz):.2f}m')
            self._sp(self.gx, self.gy, self.gz)
            self._log(t, 'SUCCESS')
            raise SystemExit(0)

        # Navigate: follow A* path
        if self.path:
            # check if current waypoint reached
            tgt = self.path[0].pose.position
            if self._d(tgt.x, tgt.y, tgt.z) < WP_TOL_M:
                self.path.pop(0)
                if not self.path:
                    if self.pose: self.hover_pose = self.pose
                    self.get_logger().info('Path exhausted, holding...')

            if self.path:
                tgt = self.path[0].pose.position
                # point camera towards the immediate target
                yaw = math.atan2(tgt.y - self.pose.position.y,
                                 tgt.x - self.pose.position.x)
                self._sp(tgt.x, tgt.y, tgt.z, yaw)
                self.hover_pose = self.pose   # keep hover updated while flying
                self._log(t, 'FLYING')
            else:
                # Path exhausted just now
                hp = self.hover_pose
                if hp: self._sp(hp.position.x, hp.position.y, hp.position.z)
                else:  self._sp(px, py, TAKEOFF_Z_M)
                self._log(t, 'HOVERING')
        else:
            # No path yet — initialise hover and wait
            if self.hover_pose is None: self.hover_pose = self.pose
            hp = self.hover_pose
            if hp: self._sp(hp.position.x, hp.position.y, hp.position.z)
            else:  self._sp(px, py, TAKEOFF_Z_M)
            self._log(t, 'HOVERING')

    def cleanup(self):
        try: self._f.flush(); self._f.close()
        except: pass


# ---------------------------------------------------------------------------
def main():
    # rclpy.init() must come first — it initialises rcl context so that
    # remove_ros_args() can correctly identify and strip ROS-specific arguments
    # from sys.argv before argparse sees them.
    rclpy.init()

    # remove_ros_args strips '--ros-args ... --' block, returning only user args.
    # This fixes the bug where argparse ignores --timeout etc. placed after '--'.
    user_argv = remove_ros_args(sys.argv[1:])

    p = argparse.ArgumentParser(description='Autonomous maze navigator')
    p.add_argument('--goal-x',  type=float, default=4.0,   help='Goal X (m)')
    p.add_argument('--goal-y',  type=float, default=7.0,   help='Goal Y (m)')
    p.add_argument('--goal-z',  type=float, default=1.5,   help='Goal Z (m)')
    p.add_argument('--timeout', type=float, default=120.0, help='Mission timeout (s)')
    p.add_argument('--csv',     default='/tmp/maze_nav.csv', help='Telemetry CSV path')
    args = p.parse_args(user_argv)

    # Log the parsed args so we can verify in the output
    import logging
    print(f'[maze_navigator] args: goal=({args.goal_x},{args.goal_y},{args.goal_z}) '
          f'timeout={args.timeout}s csv={args.csv}', flush=True)

    node = MazeNavigator(args.goal_x, args.goal_y, args.goal_z, args.timeout, args.csv)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code) if isinstance(e.code, int) else 1
    except KeyboardInterrupt:
        code = 130
    finally:
        node.cleanup()
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass
    raise SystemExit(code)


if __name__ == '__main__':
    main()
