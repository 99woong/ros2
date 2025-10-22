cmake_minimum_required(VERSION 3.8)
project(gls100_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(gls100_node src/gls100_node.cpp)

# Include directories
target_include_directories(gls100_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include  # PCAN header location
)

# Link libraries
target_link_libraries(gls100_node
  pcanbasic  # PCAN library
)

# Specify dependencies
ament_target_dependencies(gls100_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS gls100_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  # launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  # config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

