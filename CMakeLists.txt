cmake_minimum_required(VERSION 3.0.2)
project(p2_mgallai)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  script/route_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
