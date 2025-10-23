cmake_minimum_required(VERSION 3.8)
project(vslam_udp_receiver)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 실행 파일 생성
add_executable(vslam_receiver
  vslam_udp_receiver.cpp
)

# 의존성 설정
ament_target_dependencies(vslam_receiver
  rclcpp
  geometry_msgs
)

# 링크 라이브러리
target_link_libraries(vslam_receiver
  pthread
)

# 설치
install(TARGETS vslam_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# ament 패키지 설정
ament_package()