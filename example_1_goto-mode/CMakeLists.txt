cmake_minimum_required(VERSION 3.8)
project(goto-mode-example)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auterion_sdk REQUIRED)

add_executable(goto_mode src/goto_mode.cpp)

ament_target_dependencies(
  goto_mode
  auterion_sdk
)
