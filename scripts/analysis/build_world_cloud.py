#!/usr/bin/env python3
"""build_world_cloud.py

Fuses depth images, RGB frames and MAVROS poses from a rosbag into a single
colorized world point cloud (binary PLY), voxel-downsampled.

STREAMING, O(1) MEMORY DESIGN:
    Pass 1 collects poses + camera intrinsics only (kilobytes).
    Pass 2 processes each depth frame the moment it is read, keeping only a
    small rolling window of recent RGB frames for color lookup. Voxels are
    accumulated as running sums, so memory grows with scene complexity, never
    with bag length.

Usage:
    python3 build_world_cloud.py <bag_dir> [output.ply] [--stride N]

Notes:
    - Intrinsics come from /camera/color/camera_info (fallback 381.36 @ 640x480).
    - Gazebo rgbd data follows body axes (x forward, y left, z up); poses ENU.
    - NaN/no-return pixels and implausible poses (>30 m) are skipped.
"""
from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

VOXEL = 0.06
MIN_RANGE = 0.15
MAX_RANGE = 12.0
RGB_WINDOW = 6          # rolling RGB frames kept for nearest-time color
POSE_LIMIT_M = 30.0


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main() -> None:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosbag2_py import (
        ConverterOptions,
        SequentialReader,
        StorageFilter,
        StorageOptions,
    )
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo, Image

    BAG = Path(sys.argv[1])
    OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("world_cloud.ply")
    STRIDE = int(sys.argv[sys.argv.index("--stride") + 1]) if "--stride" in sys.argv else 5

    def open_reader(topics):
        r = SequentialReader()
        r.open(StorageOptions(uri=str(BAG)), ConverterOptions("cdr", "cdr"))
        r.set_filter(StorageFilter(topics=topics))
        return r

    # ---- Pass 1: poses + intrinsics (small) --------------------------------
    fx = fy = 381.36
    cx = cy = 320.0
    pose_ts_list: list[int] = []
    pose_R: list[np.ndarray] = []
    pose_T: list[np.ndarray] = []
    r = open_reader(["/uas1/local_position/pose", "/camera/color/camera_info"])
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic.endswith("/pose"):
            m = deserialize_message(data, PoseStamped)
            q, p = m.pose.orientation, m.pose.position
            pose_ts_list.append(ts)
            pose_R.append(quat_R(q.x, q.y, q.z, q.w))
            pose_T.append(np.array([p.x, p.y, p.z], np.float32))
        elif topic.endswith("camera_info"):
            cam_k = deserialize_message(data, CameraInfo).k
            if cam_k[0] > 0:
                fx, fy = float(cam_k[0]), float(cam_k[4])
                cx, cy = float(cam_k[2]), float(cam_k[5])
    print(f"pass1: {len(pose_ts_list)} poses | fx={fx:.1f} fy={fy:.1f}", flush=True)

    # ---- Pass 2: streaming fusion ------------------------------------------
    us_full = None
    voxels: dict[tuple, np.ndarray] = {}   # key -> [x,y,z, r,g,b, weight]
    rgb_window: deque[tuple[int, Image]] = deque(maxlen=RGB_WINDOW)
    fused = skipped = 0
    t0 = time.time()

    r = open_reader(["/camera/depth/image_raw", "/camera/color/image_raw"])
    while r.has_next():
        topic, data, ts = r.read_next()

        if topic == "/camera/color/image_raw":
            m = deserialize_message(data, Image)
            rgb_window.append((ts, m))
            continue

        # ---- depth frame ---------------------------------------------------
        m = deserialize_message(data, Image)
        if m.encoding in ("32FC1", "32F"):
            D = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
        elif m.encoding == "16UC1":
            D = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        else:
            continue

        i = int(np.searchsorted(pose_ts_list, ts))
        i = min(max(i, 0), len(pose_ts_list) - 1)
        R, T = pose_R[i], pose_T[i]
        if np.abs(T).max() > POSE_LIMIT_M:
            skipped += 1
            continue

        # choose nearest RGB inside the rolling window
        best = None
        for rts, rm in reversed(rgb_window):
            d = abs(rts - ts)
            if best is None or d < best[0]:
                best = (d, rm)

        if us_full is None or us_full.shape[0] != D.shape[1]:
            us_full = np.arange(0, D.shape[1], STRIDE)
            vs_full = np.arange(0, D.shape[0], STRIDE)
            uu, vv = np.meshgrid(us_full, vs_full)

        Z = D[vv, uu]
        ok = np.isfinite(Z) & (Z > MIN_RANGE) & (Z < MAX_RANGE)
        n_ok = int(ok.sum())
        if n_ok < 50:
            continue
        z = Z[ok]
        u = uu[ok].astype(np.float32)
        v = vv[ok].astype(np.float32)
        px = (u - cx) * z / fx
        py = -(v - cy) * z / fy
        pb = np.stack([z, -px, -py], axis=1)
        pw = pb @ R.T + T

        if best is not None and best[0] < 200_000_000:
            im_msg = best[1]
            im = np.frombuffer(bytes(im_msg.data), np.uint8).reshape(
                im_msg.height, im_msg.width, 3)[vv[ok], uu[ok]].astype(np.float32)
        else:
            im = np.full((n_ok, 3), 180.0, np.float32)

        keys_i = (pw / VOXEL).astype(np.int64)
        uniq_keys, inverse = np.unique(keys_i, axis=0, return_inverse=True)
        order = np.argsort(inverse, kind="stable")
        feats_sorted = np.hstack([pw, im]).astype(np.float64)[order]
        inv_sorted = inverse[order]
        starts = np.searchsorted(inv_sorted, np.arange(len(uniq_keys)))
        sums = np.add.reduceat(feats_sorted, starts, axis=0)
        counts = np.diff(np.append(starts, len(inv_sorted))).astype(np.float64)
        avg = sums / counts[:, None]
        for k, row in zip(map(tuple, uniq_keys), avg):
            e = voxels.get(k)
            if e is None:
                voxels[k] = np.array([row[0], row[1], row[2],
                                      row[3], row[4], row[5], 1.0], np.float32)
            else:
                w = e[6]
                e[0:3] += (row[0:3] - e[0:3]) / (w + 1.0)
                e[3:6] += (row[3:6] - e[3:6]) / (w + 1.0)
                e[6] = w + 1.0

        fused += 1
        if fused % 100 == 0:
            rss = int(Path(f"/proc/self/status").read_text().split("VmRSS:")[1]
                      .split("kB")[0]) // 1024
            print(f"  fused {fused} frames | voxels {len(voxels)} | RSS {rss} MB",
                  flush=True)

    print(f"fused {fused} frames ({skipped} skipped), {len(voxels)} voxels", flush=True)

    pts = np.empty((len(voxels), 3), np.float32)
    cols = np.empty((len(voxels), 3), np.uint8)
    for row, i in zip(voxels.values(), range(len(voxels))):
        pts[i] = row[0:3]
        cols[i] = np.clip(row[3:6], 0, 255).astype(np.uint8)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(pts)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n"
                b"property uchar red\nproperty uchar green\nproperty uchar blue\n"
                b"end_header\n")
        rec = np.empty(len(pts), dtype=[("xyz", "<f4", 3), ("rgb", "<u1", 3)])
        rec["xyz"] = pts
        rec["rgb"] = cols
        f.write(rec.tobytes())
    print(f"wrote {OUT}: {len(pts)} points", flush=True)


if __name__ == "__main__":
    t0 = time.time()
    main()
    print(f"done in {time.time()-t0:.0f}s")
