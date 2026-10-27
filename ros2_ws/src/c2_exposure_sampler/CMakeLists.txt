cmake_minimum_required(VERSION 3.8)
project(c2_exposure_sampler)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(time_sync_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraExposureDebug.msg"
  DEPENDENCIES std_msgs
)

add_executable(
  c2_exposure_sampler_node
  src/c2_exposure_sampler_node.cpp
)

ament_target_dependencies(
  c2_exposure_sampler_node
  rclcpp
  std_msgs
  time_sync_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  c2_exposure_sampler_node
  "${cpp_typesupport_target}"
)

install(
  TARGETS c2_exposure_sampler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
