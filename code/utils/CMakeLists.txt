CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(utils)



#Include ROS2
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(DartDemo REQUIRED)
find_package(dart_interfaces REQUIRED)  
                       
find_package(bbm_interfaces REQUIRED)

include_directories ("${PROJECT_SOURCE_DIR}/msg")
include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/incude")
include_directories (${DARTDEMO_INCLUDE_DIRS})



#Failsafe node part
add_executable( failsafe src/failsafe.cpp ) 

ament_target_dependencies(failsafe rclcpp std_msgs geometry_msgs sensor_msgs dart_interfaces)

install(TARGETS failsafe
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(failsafe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  

#Converter node part
add_executable( converter src/converter.cpp src/PID.cpp src/complementaryFilter.cpp) 

ament_target_dependencies(converter rclcpp std_msgs geometry_msgs sensor_msgs dart_interfaces)

install(TARGETS converter
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
  
#Logger node part
add_executable( logger src/logger.cpp) 

ament_target_dependencies(logger rclcpp std_msgs geometry_msgs dart_interfaces bbm_interfaces)

install(TARGETS logger
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
    
  
  
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
