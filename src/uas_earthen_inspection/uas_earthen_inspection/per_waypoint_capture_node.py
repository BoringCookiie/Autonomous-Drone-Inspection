#!/usr/bin/env python3
"""
per_waypoint_capture_node.py
Decoupled Camera Capture Node for Earthen Heritage UAV Inspection.

Author: Autonomous UAV Inspection Team
Description:
    Subscribes to the continuous UAV camera topic, but only processes, freezes,
    and publishes a frame when triggered by the drone reaching a designated coverage waypoint.
    This implements a decoupled logging strategy, eliminating motion blur and reducing
    redundant video stream compute overhead.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import json
from datetime import datetime


class PerWaypointCaptureNode(Node):
    """
    ROS2 Node for decoupled camera frame capture upon waypoint arrival.
    """

    def __init__(self):
        super().__init__('per_waypoint_capture_node')

        # Declare parameters
        self.declare_parameter('camera_rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('waypoint_reached_topic', '/uav/waypoint_reached')
        self.declare_parameter('captured_frame_topic', '/inspection/captured_frame')
        self.declare_parameter('save_captured_frames', True)
        self.declare_parameter('output_dir', 'results/raw_logs/captured_frames')

        # Retrieve parameters
        self.rgb_topic = self.get_parameter('camera_rgb_topic').value
        self.depth_topic = self.get_parameter('camera_depth_topic').value
        self.trigger_topic = self.get_parameter('waypoint_reached_topic').value
        self.out_frame_topic = self.get_parameter('captured_frame_topic').value
        self.save_frames = self.get_parameter('save_captured_frames').value
        self.output_dir = self.get_parameter('output_dir').value

        # Initialize CV Bridge & State Variables
        self.bridge = CvBridge()
        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.capture_pending = False
        self.current_waypoint_id = 0

        if self.save_frames and not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)

        # Subscribers
        self.sub_rgb = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )
        self.sub_trigger = self.create_subscription(
            Int32, self.trigger_topic, self.waypoint_reached_callback, 10
        )

        # Publisher for decoupled captured RGB frame
        self.pub_captured_frame = self.create_publisher(Image, self.out_frame_topic, 10)

        self.get_logger().info(
            f"PerWaypointCaptureNode initialized.\n"
            f"  Subscribed RGB: {self.rgb_topic}\n"
            f"  Subscribed Trigger: {self.trigger_topic}\n"
            f"  Publishing Captured Frame: {self.out_frame_topic}"
        )

    def rgb_callback(self, msg: Image):
        """Cache latest incoming continuous RGB frame."""
        self.latest_rgb_msg = msg

    def depth_callback(self, msg: Image):
        """Cache latest incoming depth frame."""
        self.latest_depth_msg = msg

    def waypoint_reached_callback(self, msg: Int32):
        """
        Triggered when UAV navigation stack signals arrival at a coverage waypoint.
        Captures the cached frame, logs metadata, and forwards frame to detection node.
        """
        self.current_waypoint_id = msg.data
        self.get_logger().info(
            f"[TRIGGER RECEIVED] Waypoint {self.current_waypoint_id} reached. Capturing frame..."
        )

        if self.latest_rgb_msg is None:
            self.get_logger().warn("Trigger received, but no RGB frame received yet!")
            return

        # Publish frame to AI detection node
        self.pub_captured_frame.publish(self.latest_rgb_msg)

        # Optionally save frame to disk
        if self.save_frames:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(self.latest_rgb_msg, desired_encoding='bgr8')
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"waypoint_{self.current_waypoint_id:03d}_{timestamp_str}.png"
                filepath = os.path.join(self.output_dir, filename)
                cv2.imwrite(filepath, cv_img)
                self.get_logger().info(f"Saved captured waypoint frame to {filepath}")
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge Conversion Failure: {str(e)}")

    # Implementation logic extension points:
    # 1. Synchronize RGB and Depth image timestamps using message_filters.TimeSynchronizer.
    # 2. Add UAV pose extraction from TF2/vehicle_pose topic at capture moment.


def main(args=None):
    rclpy.init(args=args)
    node = PerWaypointCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PerWaypointCaptureNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
