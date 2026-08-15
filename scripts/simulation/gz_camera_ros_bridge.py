#!/usr/bin/env python3
"""Bridge Gazebo Harmonic camera messages to ROS 2."""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from typing import Any

import gz.transport13 as gz_transport
from gz.msgs10 import camera_info_pb2, image_pb2
import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


IMAGE_ENCODINGS = {
    1: "mono8",
    2: "mono16",
    3: "rgb8",
    4: "rgba8",
    5: "bgra8",
    8: "bgr8",
    11: "32FC1",
    13: "32FC1",
    14: "32FC3",
    35: "rgb8",
    42: "rgb8",
}

def get_encoding(pixel_format: int, step: int, width: int) -> str:
    if pixel_format in IMAGE_ENCODINGS:
        return IMAGE_ENCODINGS[pixel_format]
    bpp = step // max(1, width)
    if bpp == 4:
        return "32FC1"
    if bpp == 1:
        return "mono8"
    return "rgb8"

def list_gazebo_topics() -> list[str]:
    result = subprocess.run(
        ["gz", "topic", "-l"],
        capture_output=True,
        text=True,
        check=False,
    )
    return sorted(line.strip() for line in result.stdout.splitlines() if line.strip())


def ros_stamp(node: Node, stamp: Any) -> RosTime:
    if stamp.sec or stamp.nsec:
        return RosTime(sec=int(stamp.sec), nanosec=int(stamp.nsec))
    return node.get_clock().now().to_msg()


def frame_id(header: Any, default: str) -> str:
    for entry in header.data:
        if entry.key in ("frame_id", "frame") and entry.value:
            return entry.value[0]
    return default


class CameraBridge(Node):
    def __init__(
        self,
        topics: list[str],
        default_frame: str,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
    ) -> None:
        super().__init__("gz_camera_bridge")
        self._default_frame = default_frame
        self._rgb_topic = rgb_topic
        self._depth_topic = depth_topic
        self._camera_info_topic = camera_info_topic
        self._gz_node = gz_transport.Node()
        self._gz_subscriptions: list[Any] = []
        self._subscribed_topics: set[str] = set()

        for topic in topics:
            self._add_topic_bridge(topic)

        # Periodic check for newly published Gazebo camera topics
        self.create_timer(3.0, self._discover_new_topics)

    def _discover_new_topics(self) -> None:
        all_topics = list_gazebo_topics()
        for topic in all_topics:
            if topic not in self._subscribed_topics:
                if topic.lower().endswith("/camera_info") or "image" in topic.lower() or topic in ("/camera", "/depth_camera"):
                    self._add_topic_bridge(topic)

    def _add_topic_bridge(self, topic: str) -> None:
        if topic in self._subscribed_topics:
            return
        self._subscribed_topics.add(topic)

        if topic.lower().endswith("/camera_info") or topic == "/camera_info":
            self._subscribe_camera_info(topic, self._camera_info_topic, "/camera_info")
        elif "image" in topic.lower() or topic in ("/camera", "/depth_camera"):
            is_depth = "depth" in topic.lower() or "disparity" in topic.lower()
            canonical = self._depth_topic if is_depth else self._rgb_topic
            alias = "/depth_camera" if is_depth else "/camera"
            self._subscribe_image(topic, canonical, alias)

    def _subscribe_image(self, topic: str, output_topic: str, alias_topic: str) -> None:
        pub_canonical = self.create_publisher(Image, output_topic, 10)
        pub_alias = self.create_publisher(Image, alias_topic, 10) if alias_topic != output_topic else None

        def callback(message: image_pb2.Image) -> None:
            try:
                encoding = get_encoding(message.pixel_format_type, message.step, message.width)

                output = Image()
                output.header.stamp = ros_stamp(self, message.header.stamp)
                output.header.frame_id = frame_id(message.header, self._default_frame)
                output.height = message.height
                output.width = message.width
                output.encoding = encoding
                output.is_bigendian = False
                output.step = message.step
                output.data = bytes(message.data)

                pub_canonical.publish(output)
                if pub_alias is not None:
                    pub_alias.publish(output)
            except Exception as e:
                self.get_logger().error(f"Image callback error on {topic}: {e}")

        try:
            sub = self._gz_node.subscribe(image_pb2.Image, topic, callback)
            self._gz_subscriptions.append(sub)
            self.get_logger().info(f"Bridged Gazebo image [{topic}] -> ROS 2 [{output_topic}] & [{alias_topic}]")
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to Gazebo topic {topic}: {e}")

    def _subscribe_camera_info(self, topic: str, output_topic: str, alias_topic: str) -> None:
        pub_canonical = self.create_publisher(CameraInfo, output_topic, 10)
        pub_alias = self.create_publisher(CameraInfo, alias_topic, 10) if alias_topic != output_topic else None

        def callback(message: camera_info_pb2.CameraInfo) -> None:
            try:
                output = CameraInfo()
                output.header.stamp = ros_stamp(self, message.header.stamp)
                output.header.frame_id = frame_id(message.header, self._default_frame)
                output.width = message.width
                output.height = message.height
                output.distortion_model = {
                    0: "plumb_bob",
                    1: "rational_polynomial",
                    2: "equidistant",
                }.get(message.distortion.model, "plumb_bob")
                output.d = list(message.distortion.k)
                output.k = list(message.intrinsics.k)
                output.p = list(message.projection.p)
                rectification = list(message.rectification_matrix)
                output.r = rectification if len(rectification) == 9 else [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                ]
                pub_canonical.publish(output)
                if pub_alias is not None:
                    pub_alias.publish(output)
            except Exception as e:
                self.get_logger().error(f"CameraInfo callback error on {topic}: {e}")

        try:
            sub = self._gz_node.subscribe(camera_info_pb2.CameraInfo, topic, callback)
            self._gz_subscriptions.append(sub)
            self.get_logger().info(f"Bridged Gazebo camera info [{topic}] -> ROS 2 [{output_topic}] & [{alias_topic}]")
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to Gazebo camera info {topic}: {e}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wait-sec", type=float, default=120.0)
    parser.add_argument("--frame-id", default="camera_link")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    args = parser.parse_args()

    default_targets = ["/camera", "/depth_camera", "/camera_info"]
    rclpy.init()
    print("[gz_camera_ros_bridge] Initialized Gazebo -> ROS 2 Camera Bridge", flush=True)
    node = CameraBridge(
        default_targets,
        args.frame_id,
        args.rgb_topic,
        args.depth_topic,
        args.camera_info_topic,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
