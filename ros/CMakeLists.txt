cmake_minimum_required(VERSION 3.0.2)
project(pallet_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  message_filters
  sensor_msgs
  geometry_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  ../scripts/run_ros1_node.py
  ../scripts/ros1_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
