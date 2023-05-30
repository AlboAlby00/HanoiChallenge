cmake_minimum_required(VERSION 3.0.2)
project(red_robot_utils)

add_compile_options(-std=c++17)

add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)


find_package(catkin REQUIRED COMPONENTS
  franka_skill
  franka_ros_example
  roscpp
  std_msgs
  dynamic_reconfigure
)



catkin_package(
   INCLUDE_DIRS include
   LIBRARIES robotSkill
   CATKIN_DEPENDS franka_ros_example franka_skill dynamic_reconfigure

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulated_robot_service src/nodes/simulated_robot_service.cpp src/simulatedRobot.cpp src/robotSkill.cpp)
target_link_libraries(simulated_robot_service ${catkin_LIBRARIES})
add_dependencies(simulated_robot_service ${catkin_EXPORTED_TARGETS})


## Declare a C++ library
add_library(robotSkill
        src/robotSkill.cpp
)
#target_include_directories(${PROJECT_NAME} PUBLIC /home/tumi6/workspace/frankx/include /home/tumi6/workspace/frankx/affx/include )
 target_link_libraries(robotSkill
   ${catkin_LIBRARIES}
 )









