cmake_minimum_required(VERSION 3.8)
project(precision-landing)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auterion_sdk REQUIRED)

add_executable(precision_landing src/precision_landing.cpp)


ament_target_dependencies(
  precision_landing
  auterion_sdk
)
