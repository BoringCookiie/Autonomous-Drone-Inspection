#!/usr/bin/env bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml
source /opt/ros/humble/setup.bash
sleep 5
until ros2 run mavros mavros_node --ros-args -r __ns:=/uas1 -p fcu_url:=udp://:14540@127.0.0.1:14580; do
  echo '[mavros] MAVROS exited, restarting in 2s...'
  sleep 2
done
