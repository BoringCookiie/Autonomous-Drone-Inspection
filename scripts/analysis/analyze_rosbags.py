#!/usr/bin/env python3
"""List and summarize ROS 2 bags recorded by fly_pattern.py.

Run inside the sim container:

  source /opt/ros/humble/setup.bash
  python3 /home/uas/scripts/analyze_rosbags.py --bag latest --export-csv --export-video

The script focuses on MAVROS/PX4 flight sensors, and it can export camera topics to videos
when image topics exist in the bag (base gz_x500 usually has no camera).
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Any, Callable

# Ensure ROS 2 python site-packages are in sys.path even if setup.bash was not sourced
ros_site_packages = '/opt/ros/humble/lib/python3.10/site-packages'
if ros_site_packages not in sys.path and Path(ros_site_packages).exists():
    sys.path.append(ros_site_packages)

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_ROOTS = [Path('/home/uas/rosbags'), Path('rosbags')]


@dataclass
class NumericStats:
    count: int = 0
    minimum: float | None = None
    maximum: float | None = None
    total: float = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    @property
    def mean(self) -> float | None:
        return None if self.count == 0 else self.total / self.count

    def text(self, unit: str = '') -> str:
        if self.count == 0 or self.minimum is None or self.maximum is None or self.mean is None:
            return 'no samples'
        suffix = f' {unit}' if unit else ''
        return f'min={self.minimum:.3f}{suffix}, max={self.maximum:.3f}{suffix}, mean={self.mean:.3f}{suffix}, n={self.count}'


@dataclass
class PositionStats:
    count: int = 0
    first: tuple[float, float, float] | None = None
    last: tuple[float, float, float] | None = None
    prev: tuple[float, float, float] | None = None
    distance_3d: float = 0.0
    x: NumericStats = field(default_factory=NumericStats)
    y: NumericStats = field(default_factory=NumericStats)
    z: NumericStats = field(default_factory=NumericStats)

    def add(self, x: float, y: float, z: float) -> None:
        point = (x, y, z)
        if self.first is None:
            self.first = point
        if self.prev is not None:
            self.distance_3d += float(np.linalg.norm(np.array(point) - np.array(self.prev)))
        self.prev = point
        self.last = point
        self.count += 1
        self.x.add(x)
        self.y.add(y)
        self.z.add(z)


@dataclass
class BagInfo:
    path: Path
    storage_id: str = 'sqlite3'
    duration_s: float = 0.0
    message_count: int = 0
    topic_counts: dict[str, int] = field(default_factory=dict)
    topic_types: dict[str, str] = field(default_factory=dict)


class CsvSink:
    def __init__(self, out_dir: Path, name: str, columns: list[str]) -> None:
        out_dir.mkdir(parents=True, exist_ok=True)
        self.path = out_dir / name
        self.file = self.path.open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(columns)

    def row(self, values: list[Any]) -> None:
        self.writer.writerow(values)

    def close(self) -> None:
        self.file.close()


class VideoSink:
    def __init__(self, out_dir: Path, topic: str, fps: float = 20.0) -> None:
        out_dir.mkdir(parents=True, exist_ok=True)
        safe = topic.strip('/').replace('/', '_') or 'camera'
        self.path = out_dir / f'{safe}.mp4'
        self.fps = fps
        self.writer: cv2.VideoWriter | None = None
        self.frames = 0

    def _ensure_writer(self, frame: np.ndarray) -> None:
        if self.writer is not None:
            return
        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(str(self.path), fourcc, self.fps, (width, height), True)

    def write(self, frame: np.ndarray) -> None:
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        self._ensure_writer(frame)
        if self.writer is not None:
            self.writer.write(frame)
            self.frames += 1

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()


class BagAnalyzer:
    def __init__(self, bag: BagInfo, export_dir: Path | None, export_csv: bool, export_video: bool) -> None:
        self.bag = bag
        self.export_dir = export_dir
        self.export_csv = export_csv
        self.export_video = export_video
        self.local_position = PositionStats()
        self.setpoints = PositionStats()
        self.velocity_local = NumericStats()
        self.velocity_body = NumericStats()
        self.imu_accel = NumericStats()
        self.imu_gyro = NumericStats()
        self.gps_alt = NumericStats()
        self.rel_alt = NumericStats()
        self.altitude_rel = NumericStats()
        self.battery_pct = NumericStats()
        self.battery_voltage = NumericStats()
        self.modes: list[tuple[float, str, bool]] = []
        self.landed_states: list[tuple[float, int]] = []
        self.status_text: list[str] = []
        self.csv_sinks: list[CsvSink] = []
        self.video_sinks: dict[str, VideoSink] = {}

    def _csv(self, name: str, columns: list[str]) -> CsvSink | None:
        if not self.export_csv or self.export_dir is None:
            return None
        sink = CsvSink(self.export_dir, name, columns)
        self.csv_sinks.append(sink)
        return sink

    def _video(self, topic: str) -> VideoSink | None:
        if not self.export_video or self.export_dir is None:
            return None
        if topic not in self.video_sinks:
            count = self.bag.topic_counts.get(topic, 0)
            duration = max(1.0, self.bag.duration_s)
            calculated_fps = count / duration if count > 0 else 20.0
            fps = max(5.0, min(30.0, calculated_fps))
            self.video_sinks[topic] = VideoSink(self.export_dir, topic, fps=fps)
        return self.video_sinks[topic]

    def analyze(self) -> None:
        msg_types = {topic: get_message(msg_type) for topic, msg_type in self.bag.topic_types.items()}
        reader = make_reader(self.bag.path, self.bag.storage_id)

        local_csv = self._csv('local_position.csv', ['time_s', 'x_m', 'y_m', 'z_m'])
        vel_csv = self._csv('velocity_local.csv', ['time_s', 'vx_mps', 'vy_mps', 'vz_mps', 'speed_mps'])
        imu_csv = self._csv('imu.csv', ['time_s', 'topic', 'accel_mps2', 'gyro_radps'])
        gps_csv = self._csv('gps.csv', ['time_s', 'lat_deg', 'lon_deg', 'alt_m'])
        state_csv = self._csv('state.csv', ['time_s', 'mode', 'armed', 'connected'])
        battery_csv = self._csv('battery.csv', ['time_s', 'percentage', 'voltage'])

        start_ns: int | None = None

        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            start_ns = timestamp_ns if start_ns is None else start_ns
            time_s = (timestamp_ns - start_ns) / 1e9
            msg_type = msg_types.get(topic)
            if msg_type is None:
                continue
            try:
                msg = deserialize_message(data, msg_type)
            except Exception:
                continue
            self._handle_msg(topic, msg, time_s, local_csv, vel_csv, imu_csv, gps_csv, state_csv, battery_csv)

        for sink in self.csv_sinks:
            sink.close()
        for sink in self.video_sinks.values():
            sink.close()

    def _handle_msg(
        self,
        topic: str,
        msg: Any,
        time_s: float,
        local_csv: CsvSink | None,
        vel_csv: CsvSink | None,
        imu_csv: CsvSink | None,
        gps_csv: CsvSink | None,
        state_csv: CsvSink | None,
        battery_csv: CsvSink | None,
    ) -> None:
        if topic.endswith('/local_position/pose'):
            p = msg.pose.position
            self.local_position.add(float(p.x), float(p.y), float(p.z))
            if local_csv:
                local_csv.row([time_s, p.x, p.y, p.z])
        elif topic.endswith('/setpoint_position/local'):
            p = msg.pose.position
            self.setpoints.add(float(p.x), float(p.y), float(p.z))
        elif topic.endswith('/local_position/velocity_local'):
            v = msg.twist.linear
            speed = norm3(v.x, v.y, v.z)
            self.velocity_local.add(speed)
            if vel_csv:
                vel_csv.row([time_s, v.x, v.y, v.z, speed])
        elif topic.endswith('/local_position/velocity_body'):
            v = msg.twist.linear
            self.velocity_body.add(norm3(v.x, v.y, v.z))
        elif topic.endswith('/imu/data') or topic.endswith('/imu/data_raw'):
            accel = msg.linear_acceleration
            gyro = msg.angular_velocity
            accel_mag = norm3(accel.x, accel.y, accel.z)
            gyro_mag = norm3(gyro.x, gyro.y, gyro.z)
            self.imu_accel.add(accel_mag)
            self.imu_gyro.add(gyro_mag)
            if imu_csv:
                imu_csv.row([time_s, topic, accel_mag, gyro_mag])
        elif topic.endswith('/global_position/global') or topic.endswith('/global_position/raw/fix'):
            self.gps_alt.add(float(msg.altitude))
            if gps_csv:
                gps_csv.row([time_s, msg.latitude, msg.longitude, msg.altitude])
        elif topic.endswith('/global_position/rel_alt'):
            self.rel_alt.add(float(msg.data))
        elif topic.endswith('/altitude'):
            self.altitude_rel.add(float(getattr(msg, 'relative', 0.0)))
        elif topic.endswith('/battery'):
            if getattr(msg, 'percentage', float('nan')) == getattr(msg, 'percentage', None):
                self.battery_pct.add(float(msg.percentage) * 100.0)
            self.battery_voltage.add(float(msg.voltage))
            if battery_csv:
                battery_csv.row([time_s, msg.percentage, msg.voltage])
        elif topic.endswith('/state'):
            entry = (time_s, str(msg.mode), bool(msg.armed))
            if not self.modes or self.modes[-1][1:] != entry[1:]:
                self.modes.append(entry)
            if state_csv:
                state_csv.row([time_s, msg.mode, msg.armed, msg.connected])
        elif topic.endswith('/extended_state'):
            state = int(msg.landed_state)
            if not self.landed_states or self.landed_states[-1][1] != state:
                self.landed_states.append((time_s, state))
        elif topic.endswith('/statustext/recv'):
            text = getattr(msg, 'text', '')
            if text:
                self.status_text.append(text)
        elif self.export_video and self.bag.topic_types.get(topic) == 'sensor_msgs/msg/Image':
            frame = image_msg_to_bgr(msg)
            sink = self._video(topic)
            if frame is not None and sink is not None:
                sink.write(frame)
        elif self.export_video and self.bag.topic_types.get(topic) == 'sensor_msgs/msg/CompressedImage':
            frame = compressed_image_to_bgr(msg)
            sink = self._video(topic)
            if frame is not None and sink is not None:
                sink.write(frame)

    def print_report(self) -> None:
        active_topics = {t: c for t, c in self.bag.topic_counts.items() if c > 0}
        image_topics = [
            (topic, self.bag.topic_counts.get(topic, 0), msg_type)
            for topic, msg_type in self.bag.topic_types.items()
            if msg_type in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
        ]

        print(f'\nBag: {self.bag.path}')
        print(f'Duration: {self.bag.duration_s:.2f}s')
        print(f'Messages: {self.bag.message_count}')
        print(f'Active topics: {len(active_topics)} / {len(self.bag.topic_types)}')

        print('\nFlight summary')
        print(f'  Modes: {format_modes(self.modes)}')
        print(f'  Landed states: {format_landed_states(self.landed_states)}')
        print(f'  Local position samples: {self.local_position.count}')
        if self.local_position.first and self.local_position.last:
            print(f'  Start xyz: {fmt_xyz(self.local_position.first)}')
            print(f'  End xyz:   {fmt_xyz(self.local_position.last)}')
            print(f'  Travel distance (local pose): {self.local_position.distance_3d:.2f} m')
            print(f'  X range: {self.local_position.x.text("m")}')
            print(f'  Y range: {self.local_position.y.text("m")}')
            print(f'  Z range: {self.local_position.z.text("m")}')
        print(f'  Local speed: {self.velocity_local.text("m/s")}')
        print(f'  Body speed: {self.velocity_body.text("m/s")}')

        print('\nSensor summary')
        print(f'  IMU accel magnitude: {self.imu_accel.text("m/s^2")}')
        print(f'  IMU gyro magnitude:  {self.imu_gyro.text("rad/s")}')
        print(f'  GPS altitude:        {self.gps_alt.text("m")}')
        print(f'  Relative altitude:   {self.rel_alt.text("m")}')
        print(f'  MAVROS altitude rel: {self.altitude_rel.text("m")}')
        print(f'  Battery percent:     {self.battery_pct.text("%")}')
        print(f'  Battery voltage:     {self.battery_voltage.text("V")}')

        print('\nMost active topics')
        for topic, count in sorted(active_topics.items(), key=lambda item: item[1], reverse=True)[:15]:
            print(f'  {count:6d}  {topic}  [{self.bag.topic_types.get(topic, "unknown")}]')

        if image_topics:
            print('\nCamera/image topics')
            for topic, count, msg_type in image_topics:
                print(f'  {count:6d}  {topic}  [{msg_type}]')
        else:
            print('\nCamera/image topics: none in this bag.')
            print('  Base gz_x500 has no camera. Use a camera model (for example gz_x500_mono_cam)')
            print('  and bridge Gazebo image topics into ROS 2 before running the pattern.')

        if self.status_text:
            print('\nStatus text')
            for text in self.status_text[:20]:
                print(f'  - {text}')

        if self.export_dir:
            print(f'\nExports: {self.export_dir}')
            if self.export_csv:
                print('  CSV files written for decoded core sensor streams.')
            if self.export_video:
                if self.video_sinks:
                    for topic, sink in self.video_sinks.items():
                        print(f'  Video for {topic}: {sink.path} ({sink.frames} frames)')
                else:
                    print('  No videos written because no image topics were present.')


def norm3(x: float, y: float, z: float) -> float:
    return float((x * x + y * y + z * z) ** 0.5)


def fmt_xyz(point: tuple[float, float, float]) -> str:
    return f'({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) m'


def format_modes(modes: list[tuple[float, str, bool]]) -> str:
    if not modes:
        return 'none recorded'
    return ' -> '.join(f'{time_s:.1f}s:{mode}{" armed" if armed else ""}' for time_s, mode, armed in modes)


def format_landed_states(states: list[tuple[float, int]]) -> str:
    labels = {0: 'undefined', 1: 'on_ground', 2: 'in_air', 3: 'takeoff', 4: 'landing'}
    if not states:
        return 'none recorded'
    return ' -> '.join(f'{time_s:.1f}s:{labels.get(state, str(state))}' for time_s, state in states)


def image_msg_to_bgr(msg: Any) -> np.ndarray | None:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding).lower()
    try:
        if encoding in ('rgb8', 'bgr8'):
            frame = data.reshape((height, width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else frame
        if encoding in ('rgba8', 'bgra8'):
            frame = data.reshape((height, width, 4))
            code = cv2.COLOR_RGBA2BGR if encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(frame, code)
        if encoding in ('mono8', '8uc1'):
            return data.reshape((height, width))
        if encoding in ('32fc1', '32fc', '32fc3'):
            depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape((height, width))
            depth_clean = np.nan_to_num(depth_data, nan=0.0, posinf=10.0, neginf=0.0)
            depth_norm = np.clip(depth_clean / 5.0 * 255.0, 0, 255).astype(np.uint8)
            return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
    except ValueError:
        return None
    return None


def compressed_image_to_bgr(msg: Any) -> np.ndarray | None:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def make_reader(path: Path, storage_id: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    return reader


def find_bags(root: Path) -> list[BagInfo]:
    if not root.exists():
        return []
    bags = []
    for metadata_path in sorted(root.glob('**/metadata.yaml')):
        bags.append(load_bag_info(metadata_path.parent))
    return sorted(bags, key=lambda bag: bag.path.stat().st_mtime, reverse=True)


def load_bag_info(path: Path) -> BagInfo:
    metadata_path = path / 'metadata.yaml'
    if metadata_path.exists():
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
        info = data.get('rosbag2_bagfile_information', data)
        duration_ns = int(info.get('duration', {}).get('nanoseconds', 0))
        topics = info.get('topics_with_message_count', [])
        topic_counts = {}
        topic_types = {}
        for item in topics:
            meta = item.get('topic_metadata', {})
            name = meta.get('name', '')
            if not name:
                continue
            topic_counts[name] = int(item.get('message_count', 0))
            topic_types[name] = meta.get('type', 'unknown')
        return BagInfo(
            path=path,
            storage_id=info.get('storage_identifier', 'sqlite3'),
            duration_s=duration_ns / 1e9,
            message_count=int(info.get('message_count', 0)),
            topic_counts=topic_counts,
            topic_types=topic_types,
        )

    reader = make_reader(path, 'sqlite3')
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return BagInfo(path=path, topic_types=topic_types)


def choose_bags(root: Path, selector: str) -> list[BagInfo]:
    bags = find_bags(root)
    if selector == 'list':
        return bags
    if selector == 'latest':
        return bags[:1]
    matches = [bag for bag in bags if bag.path.name == selector or str(bag.path) == selector]
    if matches:
        return matches
    path = Path(selector)
    if path.exists():
        return [load_bag_info(path)]
    raise SystemExit(f'Bag not found: {selector}')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='List and summarize fly_pattern ROS 2 bags.')
    parser.add_argument('--root', type=Path, default=None, help='Rosbag root directory (default /home/uas/rosbags).')
    parser.add_argument('--bag', default='latest', help='latest, list, bag directory name, or bag path.')
    parser.add_argument('--export-csv', action='store_true', help='Export decoded sensor CSV files.')
    parser.add_argument('--export-video', action='store_true', help='Export sensor_msgs/Image topics to MP4 videos.')
    parser.add_argument('--export-dir', type=Path, default=None, help='Output directory for CSV/video exports.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = args.root
    if root is None:
        root = next((candidate for candidate in DEFAULT_ROOTS if candidate.exists()), DEFAULT_ROOTS[0])

    bags = choose_bags(root, args.bag)
    if args.bag == 'list':
        if not bags:
            print(f'No bags found under {root}')
            return
        print(f'Bags under {root}:')
        for bag in bags:
            print(f'  {bag.path.name:32s}  {bag.duration_s:6.1f}s  {bag.message_count:8d} messages')
        return

    for bag in bags:
        export_dir = args.export_dir
        if export_dir is None and (args.export_csv or args.export_video):
            export_dir = bag.path / 'analysis'
        analyzer = BagAnalyzer(bag, export_dir, args.export_csv, args.export_video)
        analyzer.analyze()
        analyzer.print_report()


if __name__ == '__main__':
    main()
