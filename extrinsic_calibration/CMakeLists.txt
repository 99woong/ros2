cmake_minimum_required(VERSION 3.10)
# 프로젝트 이름 설정
project(SensorCalibration CXX)

# 1. C++ 표준 설정 (C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 2. 컴파일러 최적화 플래그 설정 (-O2)
# Release 모드에서 -O2 최적화를 사용하도록 설정
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# 3. 실행 파일 정의: sensor_calibration.cpp를 컴파일하여 sensor_calib 실행 파일 생성
add_executable(ex_cal ./src/ex_cal.cpp)

# 4. 헤더 파일 경로 추가 (-I/usr/include/eigen3)
# Eigen3 라이브러리의 헤더 경로를 추가합니다.
target_include_directories(ex_cal PUBLIC "/usr/include/eigen3")

# 참고: 대부분의 시스템에서 Eigen3는 헤더 전용 라이브러리이므로 
# 별도의 target_link_libraries 설정은 필요하지 않습니다.