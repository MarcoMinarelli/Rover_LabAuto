CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(bbm)

#Include OpenCV 
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


#Include ROS2
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(bbm_interfaces REQUIRED)

#BBM Vision Node part
add_executable( bbm_vision_node src/aruco.hpp src/aruco.cpp src/bbm_vision_node.cpp ) 
		
target_link_libraries(bbm_vision_node ${OpenCV_LIBRARIES})

ament_target_dependencies(bbm_vision_node rclcpp std_msgs geometry_msgs sensor_msgs cv_bridge bbm_interfaces)

install(TARGETS bbm_vision_node
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(bbm_vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  

#BBM Control Node part
add_executable( bbm_control_node src/bbm_control_node.cpp ) 

ament_target_dependencies(bbm_control_node rclcpp std_msgs geometry_msgs sensor_msgs bbm_interfaces)

install(TARGETS bbm_control_node
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(bbm_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 
  
  
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
