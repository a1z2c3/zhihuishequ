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
python catkin_ws/src/smart_community_semifinal/tools/test_lane_geometry.py
python catkin_ws/src/smart_community_semifinal/tools/test_plate_ocr.py
python catkin_ws/src/smart_community_semifinal/tools/test_perception_contracts.py
python catkin_ws/src/smart_community_semifinal/tools/test_obstacle_closed_loop.py
python catkin_ws/src/smart_community_semifinal/tools/test_plan_route.py
python catkin_ws/src/smart_community_semifinal/tools/test_navigation_contracts.py
python catkin_ws/src/smart_community_semifinal/tools/build_navigation_map.py \
  --layout catkin_ws/src/smart_community_semifinal/config/layout.json \
  --map-yaml catkin_ws/src/smart_community_semifinal/maps/slam_map.yaml \
  --map-pgm catkin_ws/src/smart_community_semifinal/maps/slam_map.pgm \
  --out-pgm catkin_ws/src/smart_community_semifinal/maps/navigation_map.pgm \
  --out-yaml catkin_ws/src/smart_community_semifinal/maps/navigation_map.yaml
python catkin_ws/src/smart_community_semifinal/tools/plan_route.py \
  --layout catkin_ws/src/smart_community_semifinal/config/layout.json \
  --map-yaml catkin_ws/src/smart_community_semifinal/maps/slam_map.yaml \
  --map-pgm catkin_ws/src/smart_community_semifinal/maps/slam_map.pgm \
  --out catkin_ws/src/smart_community_semifinal/config/layout_astar.json \
  --clearance 0.04 --robot-radius 0.20
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch --nodes smart_community_semifinal patrol.launch
roslaunch --nodes smart_community_semifinal navigation.launch gui:=false start_scene:=false start_perception:=false start_client:=false
echo 'Build, runtime contracts and launch parsing completed. See README for validation limits.'
