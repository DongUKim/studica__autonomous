cmake_minimum_required(VERSION 3.8)
project(studica_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RecordStart.srv"
  "srv/RecordStop.srv"
  "action/FollowRoute.action"
  "action/RelativeMove.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
