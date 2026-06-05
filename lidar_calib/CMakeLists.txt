cmake_minimum_required(VERSION 3.8)
project(lidar_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration segmentation kdtree)
find_package(Eigen3 REQUIRED)

set(DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  std_srvs
  message_filters
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(lidar_calibration_node
  src/lidar_calibration_node.cpp
)

ament_target_dependencies(lidar_calibration_node ${DEPS})

target_link_libraries(lidar_calibration_node
  ${PCL_LIBRARIES}
)

install(TARGETS lidar_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
