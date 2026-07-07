cmake_minimum_required(VERSION 3.8)
project(rtk_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# 커스텀 msg / srv 생성

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseUpdate.msg"
  "srv/SetOrigin.srv"
  "srv/SetDatum.srv"
  DEPENDENCIES std_msgs geometry_msgs
)


# 실행파일 1: heading_to_imu_node

add_executable(heading_to_imu_node
  src/heading_to_imu_node.cpp
)
ament_target_dependencies(heading_to_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(heading_to_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# 실행파일 2: gnss_corrector_node

add_executable(gnss_corrector_node
  src/gnss_corrector_node.cpp
)
ament_target_dependencies(gnss_corrector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_msgs
)
target_include_directories(gnss_corrector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gnss_corrector_node Eigen3::Eigen)

# 생성된 msg/srv 인터페이스 링크
rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(gnss_corrector_node "${cpp_typesupport_target}")


# 설치

install(TARGETS
  heading_to_imu_node
  gnss_corrector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
