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
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)


set(targets
    chassis_serial_node
  arm_serial_node
  commander_template
)

add_executable(chassis_serial_node
  src/chassis_serial_node.cpp
  src/send_command.cpp
  src/serial_port.cpp
)
target_include_directories(chassis_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(chassis_serial_node PUBLIC c_std_99 cxx_std_17)


install(TARGETS chassis_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pole_serial_node
  src/pole_serial_node.cpp
  src/send_command.cpp
  src/serial_port.cpp
)
target_include_directories(pole_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pole_serial_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS pole_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(arm_serial_node
  src/arm_serial_node.cpp
  src/send_command.cpp
  src/serial_port.cpp
)
target_include_directories(arm_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(arm_serial_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS arm_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_serial_node
  src/camera_serial_node.cpp
)
target_include_directories(camera_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(camera_serial_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS camera_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(collector_serial_node
  src/collector_serial_node.cpp
  src/send_command.cpp
  src/serial_port.cpp
)
target_include_directories(collector_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(collector_serial_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS collector_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(commander
    src/commander.cpp
)
target_include_directories(commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(commander PUBLIC c_std_99 cxx_std_17)

install(TARGETS commander
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_target_dependencies(chassis_serial_node
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  custom_interfaces
)

ament_target_dependencies(arm_serial_node
  rclcpp
  std_msgs
  example_interfaces
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  custom_interfaces
)

ament_target_dependencies(pole_serial_node
  rclcpp
  std_msgs
  custom_interfaces
)

ament_target_dependencies(camera_serial_node
  rclcpp
  std_msgs
)

ament_target_dependencies(collector_serial_node
  rclcpp
  std_msgs
)

ament_target_dependencies(commander
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  example_interfaces
  tf2
  custom_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
