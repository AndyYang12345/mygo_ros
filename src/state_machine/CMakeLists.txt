cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CUSTOM_INTERFACES_TS_TARGET "")
if(TARGET custom_interfaces::custom_interfaces__rosidl_typesupport_cpp)
  set(CUSTOM_INTERFACES_TS_TARGET custom_interfaces::custom_interfaces__rosidl_typesupport_cpp)
elseif(TARGET custom_interfaces__rosidl_typesupport_cpp)
  set(CUSTOM_INTERFACES_TS_TARGET custom_interfaces__rosidl_typesupport_cpp)
else()
  message(FATAL_ERROR "Cannot find custom_interfaces rosidl_typesupport_cpp target")
endif()

add_executable(state_machine
  src/state_machine.cpp
  src/robot_state_machine_node.cpp
  src/states/idle_state.cpp
  src/states/chassis_state.cpp
  src/states/arm_state.cpp
  src/states/menu_state.cpp
  src/states/pole_state.cpp
  src/states/vision_task_state.cpp
  src/states/ball_state.cpp
)

add_executable(arm_joint_servo_test_node
  src/arm_joint_servo_test_node.cpp
)

add_executable(menu_ui_node
  src/menu_ui_node.cpp
)
target_include_directories(state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(state_machine PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(state_machine
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  example_interfaces
  custom_interfaces
)

ament_target_dependencies(arm_joint_servo_test_node
  rclcpp
  custom_interfaces
)

ament_target_dependencies(menu_ui_node
  rclcpp
  geometry_msgs
  custom_interfaces
)

target_link_libraries(state_machine
  ${CUSTOM_INTERFACES_TS_TARGET}
)

target_link_libraries(arm_joint_servo_test_node
  ${CUSTOM_INTERFACES_TS_TARGET}
)

target_link_libraries(menu_ui_node
  ${CUSTOM_INTERFACES_TS_TARGET}
  Qt5::Widgets
)

install(TARGETS state_machine
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arm_joint_servo_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS menu_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
