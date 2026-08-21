#!/usr/bin/env python3
"""
waypoint_reached_node.py
Publish a capture trigger when the vehicle reaches a streamed setpoint.

Author: Autonomous UAV Inspection Team
Description:
    Adapts MAVROS setpoint and pose topics to trigger per-waypoint frame capture.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32


class WaypointReachedNode(Node):
    """Adapt MAVROS setpoint/pose topics to the inspection capture contract."""

    def __init__(self) -> None:
        super().__init__('waypoint_reached_node')
        self.declare_parameter('setpoint_topic', '/uas1/setpoint_position/local')
        self.declare_parameter('pose_topic', '/uas1/local_position/pose')
        self.declare_parameter('waypoint_reached_topic', '/uav/waypoint_reached')
        self.declare_parameter('position_tolerance_m', 0.45)

        self.setpoint_topic = self.get_parameter('setpoint_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.trigger_topic = self.get_parameter('waypoint_reached_topic').value
        self.tolerance = float(self.get_parameter('position_tolerance_m').value)
        self.latest_pose = None
        self.target = None
        self.waypoint_id = -1
        self.triggered = True

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, self.setpoint_topic, self.setpoint_callback, qos)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, qos)
        self.publisher = self.create_publisher(Int32, self.trigger_topic, 10)
        self.create_timer(0.1, self.check_reached)

    def setpoint_callback(self, msg: PoseStamped) -> None:
        point = msg.pose.position
        candidate = (float(point.x), float(point.y), float(point.z))
        if self.target is None or self.distance(candidate, self.target) > 0.05:
            self.target = candidate
            self.waypoint_id += 1
            self.triggered = False

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg.pose.position

    def check_reached(self) -> None:
        if self.triggered or self.target is None or self.latest_pose is None:
            return
        current = (
            float(self.latest_pose.x),
            float(self.latest_pose.y),
            float(self.latest_pose.z),
        )
        if self.distance(current, self.target) <= self.tolerance:
            message = Int32(data=self.waypoint_id)
            self.publisher.publish(message)
            self.triggered = True
            self.get_logger().info(
                f'Waypoint {self.waypoint_id} reached; capture trigger published'
            )

    @staticmethod
    def distance(first, second) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointReachedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
