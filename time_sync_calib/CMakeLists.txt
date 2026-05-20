cmake_minimum_required(VERSION 3.10)
project(time_sync_calib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(time_sync_calib
  src/main.cpp
  src/bag_reader.cpp
  src/preprocessor.cpp
  src/xcorr.cpp
  src/result_writer.cpp
)

ament_target_dependencies(time_sync_calib
  rclcpp
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
)

install(TARGETS time_sync_calib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()