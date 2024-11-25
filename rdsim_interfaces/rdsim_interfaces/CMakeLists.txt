cmake_minimum_required(VERSION 3.8)
project(rdsim_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Delivery.msg
  srv/Delivery.srv
  action/Delivery.action
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
