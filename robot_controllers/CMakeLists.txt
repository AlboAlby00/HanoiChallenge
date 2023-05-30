cmake_minimum_required(VERSION 3.0.2)
project(robot_controllers)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  franka_ros_example
  red_robot_utils
  red_msgs
)

generate_dynamic_reconfigure_options(
  cfg/RobotParams.cfg
  cfg/PickAndPlaceParams.cfg
  cfg/HanoiParams.cfg
)


catkin_package(
    CATKIN_DEPENDS franka_ros_example red_robot_utils red_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller 
  src/controller/controllerNode.cpp src/controller/controllerService.cpp src/controller/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
  pickAndPlaceController 
  src/pickAndPlaceController/pickAndPlaceControllerNode.cpp 
  src/pickAndPlaceController/pickAndPlaceControllerService.cpp 
  src/pickAndPlaceController/pickAndPlaceController.cpp
  src/controller/controller.cpp
  src/controller/controllerService.cpp
  )
target_link_libraries(pickAndPlaceController ${catkin_LIBRARIES})
add_dependencies(pickAndPlaceController ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
  hanoiController 
  src/hanoiController/hanoiControllerNode.cpp 
  src/hanoiController/hanoiControllerService.cpp 
  src/hanoiController/hanoiController.cpp
  src/pickAndPlaceController/pickAndPlaceControllerService.cpp 
  src/pickAndPlaceController/pickAndPlaceController.cpp
  src/controller/controller.cpp
  src/controller/controllerService.cpp
  )
target_link_libraries(hanoiController ${catkin_LIBRARIES})
add_dependencies(hanoiController ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)




