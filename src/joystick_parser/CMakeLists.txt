cmake_minimum_required(VERSION 3.8)
project(joystick_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(joystick_parser src/joystick_parser_node.cpp)
target_include_directories(joystick_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(joystick_parser PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(joystick_parser
  rclcpp
  sensor_msgs
  custom_interfaces
)

install(TARGETS joystick_parser
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
