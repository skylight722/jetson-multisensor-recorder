cmake_minimum_required(VERSION 3.8)
project(nvpps_ros_publisher)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(time_sync_msgs REQUIRED)

add_executable(
  nvpps_ros_publisher
  src/nvpps_ros_publisher.cpp
)

target_include_directories(
  nvpps_ros_publisher
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  nvpps_ros_publisher
  rclcpp
  time_sync_msgs
)

install(
  TARGETS nvpps_ros_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
