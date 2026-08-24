#!/usr/bin/env python3
"""octomap_to_ply.py — capture one /octomap_point_cloud_centers message and
write it as a colored binary PLY. The OctoMap server has already fused,
ray-traced and filtered the depth stream; this just exports its map.

Usage: octomap_to_ply.py [output.ply] [--timeout S] [--topic T]
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("octomap_world.ply")
    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) if "--timeout" in sys.argv else 30.0
    topic = sys.argv[sys.argv.index("--topic") + 1] if "--topic" in sys.argv \
        else "/octomap_point_cloud_centers"

    rclpy.init()
    n = Node("octomap_capture")
    got = {"m": None}

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(PointCloud2, topic, lambda m: got.__setitem__("m", m), qos)

    t0 = time.time()
    while time.time() - t0 < timeout and got["m"] is None:
        rclpy.spin_once(n, timeout_sec=0.3)

    m = got["m"]
    if m is None:
        print(f"ERROR: no message received on {topic} within {timeout}s", file=sys.stderr)
        sys.exit(1)

    pts = pc2.read_points(m, field_names=("x", "y", "z"))
    P = np.stack([np.asarray(pts["x"]), np.asarray(pts["y"]),
                  np.asarray(pts["z"])], axis=1).astype(np.float32)
    # intensity -> grayscale color so viewers shade something meaningful
    if np.ptp(P[:, 2]) > 0:
        hgt = (P[:, 2] - P[:, 2].min()) / np.ptp(P[:, 2])
    else:
        hgt = np.zeros(len(P))
    cols = np.stack([120 + 100 * hgt, 90 + 90 * hgt, 60 + 60 * hgt],
                    axis=1).astype(np.uint8)

    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(P)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n"
                b"property uchar red\nproperty uchar green\nproperty uchar blue\n"
                b"end_header\n")
        rec = np.empty(len(P), dtype=[("xyz", "<f4", 3), ("rgb", "<u1", 3)])
        rec["xyz"] = P
        rec["rgb"] = cols
        f.write(rec.tobytes())
    print(f"wrote {out}: {len(P)} points "
          f"(frame_id={m.header.frame_id}, stamp={m.header.stamp.sec})", flush=True)


if __name__ == "__main__":
    main()
