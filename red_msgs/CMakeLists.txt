cmake_minimum_required(VERSION 3.0.2)
project(red_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_service_files(
    FILES 
    hanoi/State.srv
    hanoi/Command.srv
    hanoi/DiskMovement.srv
    hanoi/DisksWidth.srv
    computer_vision/WorldPoint.srv
    computer_vision/RodsPosition.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  )


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

