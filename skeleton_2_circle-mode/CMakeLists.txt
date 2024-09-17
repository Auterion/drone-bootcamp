cmake_minimum_required(VERSION 3.8)
project(circle-mode-example)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auterion_sdk REQUIRED)

add_executable(circle_mode src/circle_mode.cpp)

ament_target_dependencies(
  circle_mode
  auterion_sdk
)
