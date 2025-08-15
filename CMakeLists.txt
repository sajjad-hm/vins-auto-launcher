cmake_minimum_required(VERSION 2.8.3)
project(vins_auto_launcher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vins_odom_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
