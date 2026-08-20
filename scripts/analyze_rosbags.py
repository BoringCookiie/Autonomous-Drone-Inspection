#!/usr/bin/env python3
"""Compatibility entry point for the migrated rosbag analyzer."""

import sys
from pathlib import Path

# Ensure ROS 2 python site-packages are accessible even without sourcing setup.bash
ros_site_packages = '/opt/ros/humble/lib/python3.10/site-packages'
if ros_site_packages not in sys.path and Path(ros_site_packages).exists():
    sys.path.append(ros_site_packages)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.analysis.analyze_rosbags import main


if __name__ == '__main__':
    main()
