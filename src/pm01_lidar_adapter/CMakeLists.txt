cmake_minimum_required(VERSION 3.8)
project(pm01_lidar_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_adapter
  src/lidar_adapter.cpp
)

ament_target_dependencies(lidar_adapter
  rclcpp
  sensor_msgs
)

install(TARGETS lidar_adapter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
