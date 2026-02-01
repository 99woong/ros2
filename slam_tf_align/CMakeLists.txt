cmake_minimum_required(VERSION 3.8)
project(slam_tf_align)

# =====================
# Compiler options
# =====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# =====================
# Find dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# =====================
# Include directories
# =====================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# =====================
# Observe node
# =====================
add_executable(slam_tf_observe_node
  src/slam_tf_observe_node.cpp
)

ament_target_dependencies(slam_tf_observe_node
  rclcpp
  geometry_msgs
  message_filters
)

target_link_libraries(slam_tf_observe_node
  yaml-cpp
)

# =====================
# Apply node
# =====================
add_executable(slam_tf_apply_node
  src/slam_tf_apply_node.cpp
)

ament_target_dependencies(slam_tf_apply_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(slam_tf_apply_node
  yaml-cpp
)

# =====================
# Install
# =====================
install(TARGETS
  slam_tf_observe_node
  slam_tf_apply_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# =====================
# Export
# =====================
ament_package()
