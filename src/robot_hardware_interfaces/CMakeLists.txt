cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


set(targets
    chassis_serial_node
)

add_executable(chassis_serial_node
  src/chassis_serial_node.cpp
)
target_include_directories(chassis_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(chassis_serial_node PUBLIC c_std_99 cxx_std_17)


install(TARGETS targets
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(chassis_serial_node
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  custom_interfaces
)

ament_package()
