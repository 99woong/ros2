cmake_minimum_required(VERSION 3.10)
project(map_generator)

# 필수 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(visualization_msgs REQUIRED)

# MQTT 라이브러리
find_library(PAHO_MQTT_CPP_LIB paho-mqttpp3)
find_library(PAHO_MQTT_C_LIB paho-mqtt3a)

if (NOT PAHO_MQTT_CPP_LIB)
  message(FATAL_ERROR "paho-mqttpp3 library not found")
endif()

if (NOT PAHO_MQTT_C_LIB)
  message(FATAL_ERROR "paho-mqtt3a library not found")
endif()
add_executable(map_generator main.cpp)

ament_target_dependencies(map_generator
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nlohmann_json
  visualization_msgs
)

target_link_libraries(map_generator
  ${PAHO_MQTT_CPP_LIB}
  ${PAHO_MQTT_C_LIB}
  Threads::Threads
)

# 🔥 설치 대상 꼭 설정해야 ros2 run 가능
install(TARGETS map_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# 패키지로 등록됨을 명시
ament_package()


# cmake_minimum_required(VERSION 3.10)
# project(fms_mqtt_publish)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(nlohmann_json REQUIRED)
# find_package(Threads REQUIRED)

# find_library(PAHO_MQTT_CPP_LIB paho-mqttpp3)
# find_library(PAHO_MQTT_C_LIB paho-mqtt3a)

# if (NOT PAHO_MQTT_CPP_LIB)
#     message(FATAL_ERROR "paho-mqttpp3 library not found")
# endif()

# if (NOT PAHO_MQTT_C_LIB)
#     message(FATAL_ERROR "paho-mqtt3a library not found")
# endif()

# add_executable(fms_mqtt_publish fms_mqtt_publish.cpp)

# ament_target_dependencies(fms_mqtt_publish
#     rclcpp
#     geometry_msgs
#     tf2
#     tf2_geometry_msgs
#     nlohmann_json
# )

# target_link_libraries(fms_mqtt_publish
#     ${PAHO_MQTT_CPP_LIB}
#     ${PAHO_MQTT_C_LIB}
#     Threads::Threads)


# install(TARGETS fms_mqtt_publish DESTINATION lib/${PROJECT_NAME})

# ament_package()