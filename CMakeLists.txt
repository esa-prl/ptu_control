cmake_minimum_required(VERSION 3.5)
project(ptu_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(include)

set(dependencies
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
  "urdf"
  "rover_msgs")


add_executable(ptu_control_node src/ptu_control.cpp)
ament_target_dependencies(ptu_control_node ${dependencies})

install(TARGETS
  ptu_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()