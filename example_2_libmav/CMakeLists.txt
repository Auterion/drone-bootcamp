cmake_minimum_required(VERSION 3.8)
project(libmav-example-mode)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(auterion_sdk REQUIRED)

add_executable(libmav_mode src/libmav_mode.cpp)

target_include_directories(libmav_mode PRIVATE ${CMAKE_SOURCE_DIR}/libmav/include)
file(GLOB MAVLINK_XML ${CMAKE_CURRENT_SOURCE_DIR}/mavlink/message_definitions/v1.0/*.xml)
file(COPY ${MAVLINK_XML} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/mavlink)

ament_target_dependencies(
  libmav_mode
  auterion_sdk
)
