#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
: "${ROS_DISTRO:?Source /opt/ros/melodic/setup.bash first}"
if [ "$ROS_DISTRO" != melodic ]; then
  echo 'This release is verified on stock ROS Melodic / Python 2.7.' >&2
  exit 1
fi
chmod +x catkin_ws/src/smart_community_semifinal/scripts/*.py
python -c 'import sys, rospy, tf, cv2, numpy, PIL; assert sys.version_info[:2] == (2, 7); print("Melodic Python 2.7 imports OK")'
python catkin_ws/src/smart_community_semifinal/tools/test_runtime.py
cd catkin_ws
catkin_make -j2 -l2
source devel/setup.bash
roslaunch --nodes smart_community_semifinal patrol.launch
echo 'Build, runtime contracts and launch parsing completed. See README.md for usage and the batch1_run6 results.'
