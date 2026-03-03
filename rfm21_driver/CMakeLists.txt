cmake_minimum_required(VERSION 3.8)
project(rfm21_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs    REQUIRED)

# ── PEAK PCAN-Basic library ───────────────────────────────────────────────────
# Header: PCANBasic.h
find_path(PCAN_INCLUDE_DIR
  NAMES PCANBasic.h
  PATHS
    /usr/include
    /usr/local/include
    /usr/include/x86_64-linux-gnu
  REQUIRED
)

# Library: libpcanbasic (preferred) or libpcan (fallback)
# /lib is the primary location on this system (ldconfig confirmed)
find_library(PCAN_LIBRARY
  NAMES pcanbasic PCANBasic pcan
  PATHS
    /lib
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
  NO_DEFAULT_PATH
)
if(NOT PCAN_LIBRARY)
  find_library(PCAN_LIBRARY NAMES pcanbasic PCANBasic pcan)
endif()

if(NOT PCAN_LIBRARY)
  message(FATAL_ERROR
    "Could NOT find PCAN library.\n"
    "Run: sudo ldconfig  then retry.\n"
    "Or specify manually: colcon build --cmake-args -DPCAN_LIBRARY=/lib/libpcanbasic.so")
endif()

message(STATUS "PCAN include : ${PCAN_INCLUDE_DIR}")
message(STATUS "PCAN library : ${PCAN_LIBRARY}")

# ── Include directories ───────────────────────────────────────────────────────
include_directories(
  include
  ${PCAN_INCLUDE_DIR}
)

# ── Static library (device + driver) ─────────────────────────────────────────
add_library(rfm21_lib STATIC
  src/pcan_driver.cpp
  src/rfm21_device.cpp
)
ament_target_dependencies(rfm21_lib rclcpp geometry_msgs std_msgs)
target_include_directories(rfm21_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCAN_INCLUDE_DIR}
)
target_link_libraries(rfm21_lib ${PCAN_LIBRARY})

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(rfm21_can_node src/rfm21_can_node.cpp)
ament_target_dependencies(rfm21_can_node rclcpp geometry_msgs std_msgs)
# List rfm21_lib BEFORE ${PCAN_LIBRARY} so the linker resolves symbols in order
target_link_libraries(rfm21_can_node
  rfm21_lib
  ${PCAN_LIBRARY}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS rfm21_can_node rfm21_lib
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ── Linting / testing ─────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
