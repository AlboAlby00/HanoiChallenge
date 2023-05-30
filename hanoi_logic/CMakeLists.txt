cmake_minimum_required(VERSION 3.0.2)
project(hanoi_logic)


find_package(catkin REQUIRED COMPONENTS
  rospy
  red_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hanoi_logic
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/hanoi_logic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/hanoi_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

