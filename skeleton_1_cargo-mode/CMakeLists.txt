cmake_minimum_required(VERSION 3.8)
project(cargo-mode-example)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auterion_sdk REQUIRED)

add_executable(cargo_mode src/cargo_mode.cpp)

ament_target_dependencies(
  cargo_mode
  auterion_sdk
)
