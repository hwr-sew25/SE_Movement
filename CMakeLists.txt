cmake_minimum_required(VERSION 3.0.2)
project(movement_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  move_base_msgs
  actionlib
  tf
  message_generation
)

## ---------- Messages ----------
add_message_files(
  FILES
  TargetPose.msg
  NavStatus.msg
  MovementTelemetry.msg
  EmergencyStop.msg
)

## ---------- Services ----------
add_service_files(
  FILES
  GoToPose.srv
)

## ---------- Generate code for msgs/srvs ----------
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    std_msgs
    move_base_msgs
    actionlib
    tf
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

