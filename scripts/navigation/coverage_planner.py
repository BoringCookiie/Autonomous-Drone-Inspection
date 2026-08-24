#!/usr/bin/env python3
"""
coverage_planner.py
Parametric Boustrophedon (Lawnmower) Coverage Path Planner.

Reads real camera intrinsics from /camera/color/camera_info to compute the
vertical field-of-view footprint at the given standoff distance, then generates
a lawnmower scan pattern with the correct yaw (pi/2, facing the +Y wall) at
each waypoint.

Published:  /planner/coverage_path  (nav_msgs/Path, one PoseStamped per waypoint with yaw)

Blueprint reference sec B - Coverage Path Planning:
    N = ceil(H / d_overlap),  d_overlap = h_FOV * (1 - rho)
    h_FOV = 2 * D * tan(vFOV / 2)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def yaw_to_quaternion(yaw_rad: float) -> dict:
    """Convert a yaw angle (rad) to a quaternion (no external deps)."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return {'x': 0.0, 'y': 0.0, 'z': sy, 'w': cy}


class CoveragePlanner(Node):
    """Publish a boustrophedon inspection path over the earthen heritage facade."""

    def __init__(self):
        super().__init__('coverage_planner')

        # ---- Facade geometry parameters ---------------------------------------
        # Wall SDF: pose (0, 5, 2), size 10 x 0.5 x 4.
        # We scan X in [facade_x_min, facade_x_max], Z in [alt_min, alt_max].
        self.declare_parameter('facade_x_min', -4.0)
        self.declare_parameter('facade_x_max',  4.0)
        self.declare_parameter('alt_min',        1.8)   # min inspection altitude (m)
        self.declare_parameter('alt_max',        3.2)   # max inspection altitude (m)
        self.declare_parameter('standoff_d',     2.5)   # distance from wall face (m)
        self.declare_parameter('wall_y',         5.0)   # wall centre Y in world (m)
        self.declare_parameter('overlap_ratio',  0.3)   # strip overlap rho

        # Fallback intrinsics (overridden once /camera_info arrives).
        # x500_depth: hFOV=1.204 rad, 640x480 -> fy approx 381.36 px
        self.declare_parameter('fallback_fy', 381.36)
        self.declare_parameter('fallback_fx', 381.36)

        # ---- LATCHED publisher -----------------------------------------------
        # TRANSIENT_LOCAL: the last published message is stored and delivered to
        # any subscriber that connects AFTER the initial publish (late-joiner).
        # Without this, path_follower misses the one-shot startup publish because
        # it subscribes after coverage_planner's 5 s timer fires.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, '/planner/coverage_path', latched_qos)

        # CameraInfo subscriber - latch intrinsics once, then cancel
        self.intrinsics_ready = False
        self.fy = self.get_parameter('fallback_fy').value
        self.fx = self.get_parameter('fallback_fx').value

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._camera_info_cb, 10
        )

        # Publish after 5 s using fallback if camera_info never arrives
        self._fallback_timer = self.create_timer(5.0, self._fallback_publish)

        self.get_logger().info(
            'CoveragePlanner ready - waiting for /camera/color/camera_info '
            '(fallback publishes after 5 s)'
        )

    # ---- CameraInfo callback -------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        if self.intrinsics_ready:
            return
        if msg.k[4] > 0.0:
            self.fy = msg.k[4]
            self.fx = msg.k[0]
            self.intrinsics_ready = True
            self.get_logger().info(
                f'[CoveragePlanner] Intrinsics from /camera_info: '
                f'fx={self.fx:.2f}  fy={self.fy:.2f}'
            )
            self._fallback_timer.cancel()
            self._publish_path()

    # ---- Fallback timer ------------------------------------------------------

    def _fallback_publish(self):
        self._fallback_timer.cancel()
        if not self.intrinsics_ready:
            self.get_logger().warn(
                f'[CoveragePlanner] No /camera_info after 5 s; '
                f'using fallback fy={self.fy:.2f}'
            )
        self._publish_path()

    # ---- Core path computation -----------------------------------------------

    def _publish_path(self):
        x_min  = self.get_parameter('facade_x_min').value
        x_max  = self.get_parameter('facade_x_max').value
        alt_lo = self.get_parameter('alt_min').value
        alt_hi = self.get_parameter('alt_max').value
        D      = self.get_parameter('standoff_d').value
        wall_y = self.get_parameter('wall_y').value
        rho    = self.get_parameter('overlap_ratio').value

        # Drone Y-position: wall_face_y - standoff
        # wall_face = wall_y - half_thickness (0.25 m) = 4.75 m
        # drone_y = 4.75 - D
        drone_y = (wall_y - 0.25) - D

        H = alt_hi - alt_lo

        # Vertical camera footprint at standoff D:
        #   h_FOV = 2 * D * (image_height/2) / fy
        # For x500_depth: image_height = 480 px
        image_height_px = 480
        h_fov_m = 2.0 * D * (image_height_px / 2.0) / self.fy
        d_overlap = h_fov_m * (1.0 - rho)

        if d_overlap <= 0.0:
            self.get_logger().error('d_overlap <= 0; check standoff/intrinsics parameters.')
            return

        N_strips = math.ceil(H / d_overlap)

        self.get_logger().info(
            f'[CoveragePlanner] H={H:.2f}m  '
            f'h_FOV={h_fov_m:.2f}m  d_overlap={d_overlap:.2f}m  '
            f'N_strips={N_strips}  drone_y={drone_y:.2f}m'
        )

        # Yaw = pi/2 rad => facing +Y (towards the wall)
        YAW = math.pi / 2.0
        q = yaw_to_quaternion(YAW)

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(N_strips):
            z = alt_lo + i * d_overlap
            # Boustrophedon: even strips left->right, odd right->left
            xs = [x_min, x_max] if i % 2 == 0 else [x_max, x_min]

            for x in xs:
                ps = PoseStamped()
                ps.header.frame_id = 'odom'
                ps.header.stamp = path.header.stamp
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(drone_y)
                ps.pose.position.z = float(z)
                ps.pose.orientation.x = q['x']
                ps.pose.orientation.y = q['y']
                ps.pose.orientation.z = q['z']
                ps.pose.orientation.w = q['w']
                path.poses.append(ps)

        self.path_pub.publish(path)
        self.get_logger().info(
            f'[CoveragePlanner] Published {len(path.poses)} waypoints '
            f'on /planner/coverage_path'
        )


def main():
    rclpy.init()
    rclpy.spin(CoveragePlanner())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
