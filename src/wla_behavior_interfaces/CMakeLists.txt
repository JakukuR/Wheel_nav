cmake_minimum_required(VERSION 3.16)
project(wla_behavior_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BehaviorTask.action" "srv/ResolveTarget.srv" "msg/TaskLease.msg"
  "srv/AgentSemanticCommand.srv" "srv/PersonCommand.srv"
  DEPENDENCIES geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
