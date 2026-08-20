#!/usr/bin/env python3
"""
gz_camera_ros_bridge.py — Reliable Gz→ROS 2 camera bridge.

Reads raw pixel data from `gz topic -e -t /camera` text output, constructs
ROS sensor_msgs/Image messages, and publishes on /camera/color/image_raw with
RELIABLE QoS — matching what ros_gz_bridge uses — so that fly_pattern.py
subscriptions receive frames.

This bypasses ros_gz_bridge entirely, avoiding the Harmonic/Fortress protobuf
type mismatch ("Unknown message type [8]") that prevents frames from flowing.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import builtin_interfaces.msg


CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

GZ_TOPIC = os.environ.get('GZ_CAMERA_TOPIC', '/camera')
ROS_TOPIC = os.environ.get('ROS_CAMERA_TOPIC', '/camera/color/image_raw')


class GzCameraBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('gz_camera_bridge')
        self._pub = self.create_publisher(Image, ROS_TOPIC, CAMERA_QOS)
        self._frame_count = 0
        self._last_log = 0.0
        self.get_logger().info(
            f'[gz_camera_bridge] Bridging {GZ_TOPIC} -> {ROS_TOPIC} '
            f'with RELIABLE QoS (text-parse mode).'
        )

    def publish_raw(self, width: int, height: int, step: int,
                    pixel_format: str, data_bytes: bytes,
                    stamp_sec: int = 0, stamp_nsec: int = 0) -> None:
        msg = Image()
        msg.header = Header()
        stamp = builtin_interfaces.msg.Time()
        stamp.sec = stamp_sec
        stamp.nanosec = stamp_nsec
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = height
        msg.width = width
        msg.encoding = pixel_format  # 'rgb8' for RGB_INT8
        msg.is_bigendian = 0
        msg.step = step
        msg.data = list(data_bytes)
        self._pub.publish(msg)
        self._frame_count += 1
        now = time.time()
        if now - self._last_log > 5.0:
            self.get_logger().info(
                f'[gz_camera_bridge] Publishing {width}x{height} {pixel_format} '
                f'frames. Total={self._frame_count}'
            )
            self._last_log = now


def parse_gz_image_stream(node: GzCameraBridgeNode, proc: subprocess.Popen) -> None:
    """
    Parse the text-format protobuf output of `gz topic -e -t /camera`.
    Each message block ends with a blank line. Fields we care about:
      width, height, step, pixel_format_type, data (raw escaped string).
    """
    width = height = step = 0
    pixel_format = 'rgb8'
    stamp_sec = stamp_nsec = 0
    in_data = False
    data_chunks: list[str] = []
    # Regex to extract the raw escaped string content from data field
    data_line_re = re.compile(r'^data:\s*"(.*)"$')
    data_cont_re = re.compile(r'^"(.*)"$')  # continuation line

    def decode_gz_escape(s: str) -> bytes:
        """Decode Gz's escaped string to bytes (handles \\nnn octal and \\xHH hex)."""
        result = bytearray()
        i = 0
        while i < len(s):
            if s[i] == '\\' and i + 1 < len(s):
                nc = s[i + 1]
                if nc == 'n':
                    result.append(ord('\n'))
                    i += 2
                elif nc == 'r':
                    result.append(ord('\r'))
                    i += 2
                elif nc == 't':
                    result.append(ord('\t'))
                    i += 2
                elif nc == '\\':
                    result.append(ord('\\'))
                    i += 2
                elif nc == '"':
                    result.append(ord('"'))
                    i += 2
                elif nc.isdigit():
                    # octal \NNN
                    end = i + 2
                    while end < i + 5 and end < len(s) and s[end].isdigit():
                        end += 1
                    result.append(int(s[i+1:end], 8) & 0xFF)
                    i = end
                elif nc == 'x':
                    # hex \xNN
                    hex_str = s[i+2:i+4]
                    result.append(int(hex_str, 16) & 0xFF)
                    i += 4
                else:
                    result.append(ord(s[i]))
                    i += 1
            else:
                result.append(ord(s[i]))
                i += 1
        return bytes(result)

    for raw_line in proc.stdout:
        try:
            line = raw_line.decode('latin-1').rstrip('\r\n')
        except Exception:
            continue

        if not line.strip():
            # End of message block — publish if we have data
            if width > 0 and height > 0 and data_chunks:
                raw_str = ''.join(data_chunks)
                raw_bytes = decode_gz_escape(raw_str)
                expected = height * step
                if len(raw_bytes) >= expected and expected > 0:
                    node.publish_raw(
                        width, height, step, pixel_format,
                        raw_bytes[:expected], stamp_sec, stamp_nsec
                    )
            # Reset
            width = height = step = 0
            pixel_format = 'rgb8'
            stamp_sec = stamp_nsec = 0
            in_data = False
            data_chunks = []
            continue

        if in_data:
            # continuation of data field (long base64/escaped string)
            m = data_cont_re.match(line.strip())
            if m:
                data_chunks.append(m.group(1))
            else:
                in_data = False
        elif line.strip().startswith('sec:'):
            try:
                stamp_sec = int(line.strip().split(':')[1].strip())
            except Exception:
                pass
        elif line.strip().startswith('nsec:'):
            try:
                stamp_nsec = int(line.strip().split(':')[1].strip())
            except Exception:
                pass
        elif line.strip().startswith('width:'):
            try:
                width = int(line.strip().split(':')[1].strip())
            except Exception:
                pass
        elif line.strip().startswith('height:'):
            try:
                height = int(line.strip().split(':')[1].strip())
            except Exception:
                pass
        elif line.strip().startswith('step:'):
            try:
                step = int(line.strip().split(':')[1].strip())
            except Exception:
                pass
        elif line.strip().startswith('pixel_format_type:'):
            pft = line.strip().split(':', 1)[1].strip()
            if pft in ('RGB_INT8', '1'):
                pixel_format = 'rgb8'
            elif pft in ('RGBA_INT8', '2'):
                pixel_format = 'rgba8'
            elif pft in ('BGR_INT8', '3'):
                pixel_format = 'bgr8'
            elif pft in ('L_INT8', '5'):
                pixel_format = 'mono8'
        elif line.strip().startswith('data:'):
            m = data_line_re.match(line.strip())
            if m:
                data_chunks.append(m.group(1))
                in_data = True  # may continue on next lines


def bridge_loop(node: GzCameraBridgeNode) -> None:
    """Spawn gz topic -e subprocess in a loop, parse and republish."""
    while rclpy.ok():
        node.get_logger().info(f'[gz_camera_bridge] Connecting to gz topic {GZ_TOPIC}...')
        proc = None
        try:
            proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', GZ_TOPIC],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
            node.get_logger().info('[gz_camera_bridge] Connected. Streaming frames...')
            parse_gz_image_stream(node, proc)
        except Exception as e:
            node.get_logger().error(f'[gz_camera_bridge] Error: {e}')
        finally:
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
        if rclpy.ok():
            node.get_logger().warn('[gz_camera_bridge] Stream ended. Reconnecting in 2s...')
            time.sleep(2.0)


def main() -> None:
    rclpy.init()
    node = GzCameraBridgeNode()

    # Bridge runs in background thread; ROS spins in main thread
    t = threading.Thread(target=bridge_loop, args=(node,), daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
