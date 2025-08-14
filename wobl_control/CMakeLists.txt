cmake_minimum_required(VERSION 3.8)
project(wobl_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(wobl_messages REQUIRED)

# Include dirs
include_directories(include)

# Define the plugin
add_executable(pid_balance_controller src/pid_balance_controller_node.cpp)

ament_target_dependencies(pid_balance_controller
  rclcpp
  sensor_msgs
  control_toolbox
  wobl_messages
)

install(TARGETS
  pid_balance_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
