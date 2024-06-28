cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_merger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf2_ros
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/pointcloud_merger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)