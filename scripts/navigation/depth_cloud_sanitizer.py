#!/usr/bin/env python3
"""
depth_cloud_sanitizer.py
Makes Gazebo depth point clouds safe for octomap_server.

Problem:
    ros_gz_bridge forwards the rgbd_camera PointCloudPacked with is_dense=true,
    but roughly half the pixels are NaN (no-return rays). octomap_server trusts
    the is_dense flag, skips NaN validation, and every insertion silently fails
    -> empty obstacle map -> A* plans blind.

Fix:
    Drops non-finite rows and republishes a genuinely dense x/y/z cloud on
    /points_clean, preserving frame_id and the original sim-time stamp.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField


class DepthCloudSanitizer(Node):

    def __init__(self):
        super().__init__('depth_cloud_sanitizer')
        self.declare_parameter('input_topic', '/points')
        self.declare_parameter('output_topic', '/points_clean')
        # Empty means preserve the Gazebo frame. Relabelling a cloud without
        # transforming its coordinates corrupts the map.
        self.declare_parameter('output_frame_id', '')
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10
        )
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._cb, 10
        )
        self.get_logger().info(
            f'DepthCloudSanitizer: {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value} (NaN rows dropped)'
        )

    def _cb(self, msg: PointCloud2):
        try:
            pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
            arr = np.stack([
                np.asarray(pts['x'], dtype=np.float32),
                np.asarray(pts['y'], dtype=np.float32),
                np.asarray(pts['z'], dtype=np.float32),
            ], axis=1)
            finite = np.all(np.isfinite(arr), axis=1)
            clean = arr[finite]
            if clean.shape[0] == 0:
                return

            out = PointCloud2()
            out.header = msg.header
            output_frame = self.get_parameter('output_frame_id').value
            if output_frame:
                out.header.frame_id = output_frame
            out.height = 1
            out.width = int(clean.shape[0])
            out.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            out.is_bigendian = False
            out.point_step = 12
            out.row_step = 12 * out.width
            out.data = clean.tobytes()
            out.is_dense = True
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Sanitizer error: {e}')


def main():
    rclpy.init()
    node = DepthCloudSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
