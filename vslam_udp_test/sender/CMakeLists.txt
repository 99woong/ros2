cmake_minimum_required(VERSION 3.8)
project(vslam_udp_sender)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
add_compile_options(-Wall -Wextra -Wpedantic)

# 실행 파일 생성
add_executable(vslam_sender
  vslam_udp_sender.cpp
)

# 링크 라이브러리
target_link_libraries(vslam_sender
  pthread
)

# 설치
install(TARGETS vslam_sender
  DESTINATION bin
)