cmake_minimum_required(VERSION 3.0.2)
project(smart_community_semifinal)
find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
catkin_package(LIBRARIES semifinal_signal)
set(CMAKE_CXX_STANDARD 11)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(semifinal_signal SHARED src/signal_plugin.cpp)
target_link_libraries(semifinal_signal ${GAZEBO_LIBRARIES})
# Runtime nodes support the stock Python 2.7 ROS Melodic environment.
install(PROGRAMS scripts/official_perception_node.py scripts/traffic_guard_node.py scripts/signal_perception_node.py scripts/patrol_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/reference_detector.py scripts/semifinal_core.py scripts/traffic_detector_core.py scripts/runtime_compat.py scripts/patrol_core.py scripts/scene_geometry.py scripts/image_geometry.py scripts/signal_locator.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS semifinal_signal LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config models worlds assets urdf maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
