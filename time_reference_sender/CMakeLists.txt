cmake_minimum_required(VERSION 3.8)
project(time_reference_sender)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(time_reference_sender src/time_reference_sender_node.cpp)

ament_target_dependencies(time_reference_sender
  rclcpp
  nmea_msgs
  sensor_msgs
)

install(TARGETS
  time_reference_sender
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()