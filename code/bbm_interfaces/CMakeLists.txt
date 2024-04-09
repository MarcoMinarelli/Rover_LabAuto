CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(bbm_interfaces)

#Include ROS2
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Lineparams.msg"
 )

ament_package()
