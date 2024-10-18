cmake_minimum_required(VERSION 3.8)
project(circle-2b-mode-example)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auterion_sdk REQUIRED)

add_executable(circle_2b_mode src/circle_2b_mode.cpp)

ament_target_dependencies(
  circle_2b_mode
  auterion_sdk
)
