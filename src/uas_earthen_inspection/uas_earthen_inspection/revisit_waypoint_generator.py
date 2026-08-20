#!/usr/bin/env python3
"""
revisit_waypoint_generator.py
Confidence-Triggered Revisit Waypoint Generator Node for Person 3 (UAV / ROS2 Lead).

Author: Person 3 (UAV / ROS2 & Sim Lead)
Description:
    Subscribes to AI defect detections and camera depth maps. When a detection confidence C
    falls within an ambiguous threshold band (e.g., 0.4 <= C <= 0.7), it unprojects the 2D
    bounding box center to a 3D target coordinate (X, Y, Z), computes a dynamically closer
    revisit standoff distance Drevisit = max(Dmin, Dbase - k*(1 - C)), and publishes a new
    revisit waypoint queue to the A* navigation planner.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math


class RevisitWaypointGenerator(Node):
    """
    ROS2 Node for dynamic, confidence-triggered 3D revisit waypoint generation.
    """

    def __init__(self):
        super().__init__('revisit_waypoint_generator')

        # Declare parameters
        self.declare_parameter('flight_strategy', 'revisit') # Options: single_pass | revisit
        self.declare_parameter('detections_topic', '/inspection/detections')
        self.declare_parameter('camera_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('revisit_waypoints_topic', '/planner/revisit_waypoints')
        
        # Ambiguity threshold band
        self.declare_parameter('confidence_ambiguity_min', 0.4)
        self.declare_parameter('confidence_ambiguity_max', 0.7)

        # Standoff distance parameters (D_revisit = max(D_min, D_base - k * (1 - C)))
        self.declare_parameter('d_base', 3.0)      # Nominal flight standoff distance in meters
        self.declare_parameter('d_min', 1.0)       # Minimum safe standoff clearance in meters
        self.declare_parameter('k_standoff', 2.0)  # Uncertainty scaling coefficient

        # Camera Intrinsic Matrix Parameters (K)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        # Retrieve parameters
        self.strategy = self.get_parameter('flight_strategy').value
        self.det_topic = self.get_parameter('detections_topic').value
        self.depth_topic = self.get_parameter('camera_depth_topic').value
        self.revisit_topic = self.get_parameter('revisit_waypoints_topic').value

        self.c_min = self.get_parameter('confidence_ambiguity_min').value
        self.c_max = self.get_parameter('confidence_ambiguity_max').value
        self.d_base = self.get_parameter('d_base').value
        self.d_min = self.get_parameter('d_min').value
        self.k = self.get_parameter('k_standoff').value

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        self.bridge = CvBridge()
        self.latest_depth_img = None

        # Subscriptions
        self.sub_detections = self.create_subscription(
            Detection2DArray, self.det_topic, self.detections_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )

        # Publisher for revisit waypoints to A* planner
        self.pub_revisit_path = self.create_publisher(
            PoseArray, self.revisit_topic, 10
        )

        self.get_logger().info(
            f"RevisitWaypointGenerator operational.\n"
            f"  Strategy Mode: [{self.strategy.upper()}]\n"
            f"  Ambiguity Band: [{self.c_min:.2f}, {self.c_max:.2f}]\n"
            f"  Revisit Topic: {self.revisit_topic}"
        )

    def depth_callback(self, msg: Image):
        """Caches depth map for 3D coordinate unprojection."""
        try:
            # Handle float32 depth map encoding
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth CV Bridge error: {e}")

    def unproject_2d_to_3d(self, u: float, v: float, depth: float):
        """
        Unprojects 2D image coordinate (u, v) with depth Z into 3D camera coordinate frame (X, Y, Z).
        Formula:
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            Z = depth
        """
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return x, y, z

    def compute_revisit_standoff(self, confidence: float) -> float:
        """
        Computes dynamic revisit standoff distance:
            D_revisit = max(D_min, D_base - k * (1 - C))
        """
        d_revisit = self.d_base - self.k * (1.0 - confidence)
        return max(self.d_min, d_revisit)

    def detections_callback(self, msg: Detection2DArray):
        """Processes AI detections, filters ambiguous confidence, and publishes 3D revisit poses."""
        if self.strategy != 'revisit':
            self.get_logger().info("Flight strategy set to 'single_pass'. Revisit generation bypassed.")
            return

        revisit_poses = PoseArray()
        revisit_poses.header = msg.header

        for det in msg.detections:
            if not det.results:
                continue

            conf = det.results[0].score
            class_name = det.results[0].hypothesis.class_id
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y

            # Check if confidence falls within the ambiguous band [C_min, C_max]
            if self.c_min <= conf <= self.c_max:
                self.get_logger().info(
                    f"[AMBIGUOUS DETECTION TRIGGERED] Class: '{class_name}' | Confidence: {conf:.2f} "
                    f"in band [{self.c_min:.2f}, {self.c_max:.2f}]"
                )

                # Extract depth Z at bounding box center
                if self.latest_depth_img is not None:
                    u_idx = int(clamp(u, 0, self.latest_depth_img.shape[1] - 1))
                    v_idx = int(clamp(v, 0, self.latest_depth_img.shape[0] - 1))
                    depth_z = float(self.latest_depth_img[v_idx, u_idx])

                    if math.isnan(depth_z) or depth_z <= 0.0:
                        depth_z = 3.0  # Fallback distance if depth pixel invalid
                else:
                    depth_z = 3.0

                # 1. Unproject 2D bbox to 3D target coordinates
                target_x, target_y, target_z = self.unproject_2d_to_3d(u, v, depth_z)

                # 2. Calculate closer standoff distance
                d_revisit = self.compute_revisit_standoff(conf)

                # 3. Calculate revisit waypoint pose (offset towards origin by standoff distance)
                scale = d_revisit / max(0.001, target_z)
                revisit_x = target_x * scale
                revisit_y = target_y * scale
                revisit_z = target_z * scale

                pose = Pose()
                pose.position.x = revisit_x
                pose.position.y = revisit_y
                pose.position.z = revisit_z
                pose.orientation.w = 1.0

                revisit_poses.poses.append(pose)

                self.get_logger().info(
                    f"  -> Generated 3D Revisit Waypoint:\n"
                    f"     Target 3D: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})\n"
                    f"     Standoff D_revisit: {d_revisit:.2f}m\n"
                    f"     Revisit Pose: ({revisit_x:.2f}, {revisit_y:.2f}, {revisit_z:.2f})"
                )

        if revisit_poses.poses:
            self.pub_revisit_path.publish(revisit_poses)
            self.get_logger().info(
                f"Published {len(revisit_poses.poses)} revisit waypoint(s) to A* Planner ({self.revisit_topic})."
            )


def clamp(n, minn, maxn):
    return max(minn, min(n, maxn))


def main(args=None):
    rclpy.init(args=args)
    node = RevisitWaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RevisitWaypointGenerator.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
