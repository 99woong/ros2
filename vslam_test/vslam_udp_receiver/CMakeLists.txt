cmake_minimum_required(VERSION 3.8)
project(vslam_udp_receiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executable
add_executable(vslam_udp_receiver src/vslam_udp_receiver.cpp)

# Specify dependencies
ament_target_dependencies(vslam_udp_receiver
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install executables
install(TARGETS
  vslam_udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# # Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
