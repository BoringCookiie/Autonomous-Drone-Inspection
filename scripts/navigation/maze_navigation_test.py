#!/usr/bin/env python3
"""
maze_navigation_test.py — Full navigation test on the obstacle_maze world.

Mission:    Drone starts at (0,0,0) inside the maze.
            Goal is (4.0, 7.0, 1.5) — outside the maze through the north gap.
            A* planner must find a path through the serpentine corridors
            and the path_follower must execute it.

Maze geometry (obstacle_maze.sdf):
    Outer boundary: x∈[-5.75, 7.75], y∈[-4.75, 4.75], walls are 3m tall
    Exit gap in north wall: x∈[0, 3] at y≈4.75 (north_wall_left ends at x=0,
    north_wall_right starts at x=3)
    Inner walls create a spiral path with ~2m gaps.

Usage (inside the sim container):
    python3 /home/uas/scripts/navigation/maze_navigation_test.py \
        --goal-x 4.0 --goal-y 7.0 --goal-z 1.5 \
        --timeout 300 --trajectory-csv /home/uas/rosbags/maze_trajectory.csv
"""
import argparse
import csv
import math
import os
import sys
import time

ros_site_packages = "/opt/ros/humble/lib/python3.10/site-packages"
if ros_site_packages not in sys.path and os.path.exists(ros_site_packages):
    sys.path.append(ros_site_packages)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class MazeNavigationTest(Node):
    """Publishes a goal and monitors the drone's trajectory through the maze."""

    def __init__(self, goal_x: float, goal_y: float, goal_z: float,
                 timeout: float, csv_path: str):
        super().__init__('maze_navigation_test')

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z
        self.timeout = timeout
        self.csv_path = csv_path
        csv_dir = os.path.dirname(csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        self.start_time = None
        self.pose = None
        self.state = None
        self.battery = None
        self.latest_path = None
        self.goal_sent = False
        self.reached_goal = False
        self.min_dist_to_goal = float('inf')
        self.max_speed = 0.0
        self.total_distance = 0.0
        self.last_log_pos = None
        self._last_status = None

        # CSV logging
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_s', 'x_m', 'y_m', 'z_m',
            'dist_to_goal_m', 'speed_mps', 'mode', 'armed',
            'path_len', 'status'
        ])

        # Subscriptions — MAVROS uses BEST_EFFORT for sensor streams
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseStamped, '/uas1/local_position/pose', self._pose_cb, qos_be)
        self.create_subscription(
            State, '/uas1/state', self._state_cb, qos_be)
        self.create_subscription(
            BatteryState, '/uas1/battery', self._battery_cb, qos_be)
        self.create_subscription(
            Path, '/planned_path', self._path_cb, 10)
        self.create_subscription(
            String, '/path_follower/status', self._status_cb, 10)

        # Goal publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/navigation/goal', 10)

        # Monitoring timer — 10 Hz
        self.create_timer(0.1, self._monitor_cb)

        self.get_logger().info(
            f'MazeNavigationTest ready. Goal: ({goal_x}, {goal_y}, {goal_z}) '
            f'timeout={timeout}s'
        )

    def _pose_cb(self, msg: PoseStamped):
        self.pose = msg.pose

    def _state_cb(self, msg: State):
        self.state = msg

    def _battery_cb(self, msg: BatteryState):
        self.battery = msg

    def _path_cb(self, msg: Path):
        self.latest_path = msg

    def _status_cb(self, msg: String):
        if msg.data and msg.data != self._last_status and not self.reached_goal:
            self.get_logger().info(f'[Status] {msg.data}')
            self._last_status = msg.data

    def _send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.goal_x)
        msg.pose.position.y = float(self.goal_y)
        msg.pose.position.z = float(self.goal_z)

        # Repeat only to cover late DDS discovery. planner_3d deduplicates the
        # coordinates, so this can never create duplicate follower legs.
        if self.goal_pub.get_subscription_count() < 1:
            self.get_logger().warn(
                'No planner subscriber yet; goal will be retried on the next monitor tick.'
            )
            return
        for _ in range(3):
            self.goal_pub.publish(msg)
        self.goal_sent = True
        self.get_logger().info(
            f'>>> GOAL SENT: ({self.goal_x}, {self.goal_y}, {self.goal_z})'
        )

    def _monitor_cb(self):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
            return

        elapsed = now - self.start_time

        # Wait for pose before sending goal
        if not self.goal_sent:
            if self.pose is None:
                if int(elapsed) % 5 == 0:
                    self.get_logger().info(
                        f'Waiting for pose... ({elapsed:.0f}s)')
                return
            if elapsed < 8.0:
                return
            self._send_goal()
            return

        if self.pose is None:
            return

        px = self.pose.position.x
        py = self.pose.position.y
        pz = self.pose.position.z
        dist = math.sqrt(
            (px - self.goal_x) ** 2 +
            (py - self.goal_y) ** 2 +
            (pz - self.goal_z) ** 2
        )
        self.min_dist_to_goal = min(self.min_dist_to_goal, dist)

        # Speed and distance tracking
        speed = 0.0
        if self.last_log_pos is not None:
            lx, ly, lz, lt = self.last_log_pos
            dt = elapsed - lt
            if dt > 0:
                dx = px - lx
                dy = py - ly
                dz = pz - lz
                step = math.sqrt(dx * dx + dy * dy + dz * dz)
                speed = step / dt
                self.total_distance += step
                self.max_speed = max(self.max_speed, speed)
        self.last_log_pos = (px, py, pz, elapsed)

        mode = self.state.mode if self.state else 'unknown'
        armed = self.state.armed if self.state else False
        path_len = len(self.latest_path.poses) if self.latest_path else 0

        # CSV row
        self.csv_writer.writerow([
            f'{elapsed:.2f}',
            f'{px:.4f}', f'{py:.4f}', f'{pz:.4f}',
            f'{dist:.4f}',
            f'{speed:.4f}',
            mode,
            armed,
            path_len,
            'FLYING' if dist > 1.0 else 'CLOSE'
        ])

        # Periodic status
        if int(elapsed * 10) % 50 == 0:
            self.get_logger().info(
                f't={elapsed:.0f}s pos=({px:.2f},{py:.2f},{pz:.2f}) '
                f'dist_to_goal={dist:.2f}m speed={speed:.2f}m/s '
                f'mode={mode} path_remaining={path_len}'
            )

        # Check goal reached
        if dist < 1.0:
            self.reached_goal = True
            self.get_logger().info(
                f'>>> GOAL REACHED at t={elapsed:.1f}s! '
                f'pos=({px:.2f},{py:.2f},{pz:.2f})'
            )
            raise SystemExit(0)

        # Timeout
        if elapsed > self.timeout:
            self.get_logger().warn(
                f'TIMEOUT after {elapsed:.0f}s. '
                f'Min dist to goal: {self.min_dist_to_goal:.2f}m. '
                f'Total distance flown: {self.total_distance:.2f}m'
            )
            raise SystemExit(1 if self.min_dist_to_goal > 3.0 else 0)

    def cleanup(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()


def parse_args():
    parser = argparse.ArgumentParser(description='Maze navigation test')
    parser.add_argument('--goal-x', type=float, default=4.0)
    parser.add_argument('--goal-y', type=float, default=7.0)
    parser.add_argument('--goal-z', type=float, default=1.5)
    parser.add_argument('--timeout', type=float, default=300.0)
    parser.add_argument(
        '--trajectory-csv',
        default='/home/uas/rosbags/maze_trajectory.csv'
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MazeNavigationTest(
        args.goal_x, args.goal_y, args.goal_z,
        args.timeout, args.trajectory_csv
    )
    exit_code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        exit_code = int(exc.code) if isinstance(exc.code, int) else 1
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
