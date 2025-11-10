cmake_minimum_required(VERSION 3.8)
# 프로젝트 이름을 gls100_driver로 통일
project(gls100_driver) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(SOURCE_FILES
  src/generic_can_node.cpp
  src/pcan_driver.cpp
  src/gls100_device.cpp
)

add_executable(generic_can_node ${SOURCE_FILES})

target_include_directories(generic_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(generic_can_node
  pcanbasic
  # m 
)

ament_target_dependencies(generic_can_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS generic_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()