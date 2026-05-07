cmake_minimum_required(VERSION 3.8)
project(gnss_fusion)

# C++17 표준 사용
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

# ament 및 의존 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================
# 실행파일 1: heading_to_imu_node
# /heading (QuaternionStamped) → /gps/imu (Imu) 변환
# ============================================================
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

# ============================================================
# 실행파일 2: gnss_corrector_node
# LIO 장기 드리프트를 Dual RTK GNSS로 보정 (3-state drift EKF)
# ============================================================
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
)
target_include_directories(gnss_corrector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gnss_corrector_node Eigen3::Eigen)

# ============================================================
# 인클루드 디렉토리
# ============================================================
target_include_directories(heading_to_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================================
# 설치
# ============================================================
install(TARGETS
  heading_to_imu_node
  gnss_corrector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()