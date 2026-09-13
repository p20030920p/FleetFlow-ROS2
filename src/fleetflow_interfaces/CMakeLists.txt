cmake_minimum_required(VERSION 3.8)
project(fleetflow_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TransportTask.msg" "msg/MachineState.msg" "msg/RobotStatus.msg"
  "msg/TrafficDirective.msg"
  "srv/RequestTask.srv" "srv/AcquireLease.srv" "srv/ReleaseLease.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
