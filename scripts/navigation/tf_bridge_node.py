#!/usr/bin/env python3
"""
tf_bridge_node.py
Minimal TF bridge for the PX4/Gazebo inspection stack (no URDF required).

Problem it solves:
    The Gazebo depth bridge publishes /points with the sensor SDF frame name
    ('x500_depth_0/rgbd_camera_link/rgbd_camera'). No robot_state_publisher /
    URDF exists in this stack, so nobody publishes odom->base_link or any
    camera-frame transform. octomap_server's message filter therefore drops
    every cloud ("queue is full") and A* plans against an empty obstacle set.

What it publishes:
    odom -> base_link                                dynamic, from MAVROS pose
    base_link -> depth_camera                        static, SDF joint offset

    The identity rotation is deliberate: the gz point payload is already in
    body-axis convention (x forward, y left, z up), verified empirically
    against known maze geometry (pillar at (1.2, 0.4)).

Timing note:
    Gazebo publishes /clock and all ROS nodes consume it through use_sim_time.
    Dynamic transforms therefore use this node's simulation clock directly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TfBridge(Node):

    def __init__(self):
        super().__init__('tf_bridge')
        self.declare_parameter('pose_topic', '/uas1/local_position/pose')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'depth_camera')
        self.declare_parameter('cam_x', 0.12)
        self.declare_parameter('cam_y', 0.03)
        self.declare_parameter('cam_z', 0.242)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish the complete fixed chain as one latched TFMessage.
        base = self.get_parameter('base_frame').value
        camera_frame = self.get_parameter('camera_frame').value
        self.static_transforms = [
            self._make_static(base, camera_frame,
                              self.get_parameter('cam_x').value,
                              self.get_parameter('cam_y').value,
                              self.get_parameter('cam_z').value),
        ]
        self.static_broadcaster.sendTransform(self.static_transforms)
        # Re-latch periodically as protection for late subscribers.
        self.create_timer(
            5.0, lambda: self.static_broadcaster.sendTransform(self.static_transforms)
        )

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self._pose_cb, qos_be
        )
        self.get_logger().info(
            f'TfBridge ready: {self.get_parameter("odom_frame").value}->{base} '
            f'(dynamic, sim-time aligned) + static chain to '
            f'{self.get_parameter("camera_frame").value}'
        )

    def _make_static(self, parent: str, child: str, x: float, y: float, z: float) -> TransformStamped:
        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = 1.0
        return t

    def _pose_cb(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('base_frame').value
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
