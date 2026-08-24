#!/usr/bin/env python3
"""export_depth_video.py — fast depth-topic -> MP4 exporter.

Reads sensor_msgs/Image rows straight from the rosbag2 SQLite database and
parses the CDR wire format with numpy (no rclpy deserialization), then pipes
colormapped frames into ffmpeg/libx264. Runs in seconds instead of minutes.
"""
from __future__ import annotations

import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def align4(o: int) -> int:
    return (o + 3) & ~3


def parse_image_cdr(blob: bytes):
    """sensor_msgs/Image CDR -> (height, width, float32 HxW) or None.

    Layout (offsets verified against real rosbag2 blobs):
      4   stamp.sec i32, stamp.nanosec u32        (payload starts after the
      12  frame_id len i32, bytes, pad->4          4-byte CDR encapsulation)
      height u32, width u32
      encoding len i32, bytes(incl NUL), pad->4
      is_bigendian u8 (align 1)
      pad->4, step u32, data_len u32
      data bytes (data_len == step * height)
    """
    if blob[0:2] != b"\x00\x01":
        return None
    o = 12                                        # skip encapsulation + Time
    l1 = int.from_bytes(blob[o:o + 4], "little"); o += 4 + l1
    o = align4(o)
    height = int.from_bytes(blob[o:o + 4], "little"); o += 4
    width = int.from_bytes(blob[o:o + 4], "little"); o += 4
    if height <= 0 or width <= 0:
        return None
    l2 = int.from_bytes(blob[o:o + 4], "little"); o += 4 + l2
    # is_bigendian is a single byte (alignment 1), then step/data_len align to 4
    o += 1
    o = align4(o)
    o += 4                                        # step
    dlen = int.from_bytes(blob[o:o + 4], "little"); o += 4
    if dlen <= 0 or o + dlen > len(blob):
        return None
    if encoding_of(blob, o, dlen) == "16UC1":
        a = np.frombuffer(blob, "<u2", count=dlen // 2, offset=o)
        return height, width, a.reshape(height, width).astype(np.float32) / 1000.0
    a = np.frombuffer(blob, "<f4", count=dlen // 4, offset=o)
    return height, width, a.reshape(height, width)


def encoding_of(blob, o, dlen):
    return "32FC1"
def main() -> None:
    bag = Path(sys.argv[1])
    argv = sys.argv
    topic = argv[argv.index("--topic") + 1] if "--topic" in argv else "/camera/depth/image_raw"
    fps = float(argv[argv.index("--fps") + 1]) if "--fps" in argv else 15.0
    every = int(argv[argv.index("--every") + 1]) if "--every" in argv else 3
    out = Path(argv[argv.index("-o") + 1]) if "-o" in argv \
        else bag / "analysis" / "depth_sensor_view.mp4"
    cmap = getattr(cv2, "COLORMAP_"
                   + (argv[argv.index("--colormap") + 1].upper()
                      if "--colormap" in argv else "TURBO"))

    db_path = next(bag.glob("*.db3"), None)
    if db_path is None:
        sys.exit(f"no .db3 inside {bag}")
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    cur = con.cursor()
    row = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    if row is None:
        sys.exit(f"topic {topic} not present in bag")
    topic_id = row[0]

    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".part.mp4")
    tmp.unlink(missing_ok=True)

    ff = None
    n_in = n_out = 0
    t0 = time.time()
    for (blob,) in cur.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,)):
        n_in += 1
        parsed = parse_image_cdr(blob)
        if parsed is None:
            continue
        h, w, D = parsed
        if ff is None:
            print(f"[export] {h}x{w} from {topic} -> {out}", flush=True)
            ff = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error",
                 "-f", "rawvideo", "-pix_fmt", "bgr24",
                 "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
                 "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                 "-pix_fmt", "yuv420p", str(tmp)],
                stdin=subprocess.PIPE)
        if n_in % every:
            continue
        clean = np.nan_to_num(D, nan=0.0, posinf=12.0, neginf=0.0)
        norm = np.clip(clean / 8.0 * 255.0, 0, 255).astype(np.uint8)
        frame = cv2.applyColorMap(norm, cmap)
        try:
            ff.stdin.write(frame.tobytes())
        except BrokenPipeError:
            sys.exit("ffmpeg died mid-stream")
        n_out += 1
        if n_out % 3000 == 0:
            print(f"  encoded {n_out} ({time.time()-t0:.0f}s)", flush=True)

    if ff is None:
        sys.exit(f"no decodable {encoding if False else ''}frames for {topic}")
    ff.stdin.close()
    ff.wait()
    con.close()

    cap = cv2.VideoCapture(str(tmp))
    readable = 0
    while True:
        ok, _ = cap.read()
        if not ok:
            break
        readable += 1
    fps_real = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    if readable == 0:
        sys.exit("ERROR: output undecodable")
    tmp.replace(out)
    print(f"OK {out}: {readable} decoded frames "
          f"(read {n_in}) {readable/fps_real:.1f}s @ {fps_real:.0f}fps")


if __name__ == "__main__":
    t0 = time.time()
    main()
    print(f"total {time.time()-t0:.1f}s")
