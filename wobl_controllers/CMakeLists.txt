cmake_minimum_required(VERSION 3.8)
project(wobl_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include dirs
include_directories(
  include
)

# Define the plugin
add_library(pid_balance_controller SHARED
  src/pid_balance_controller.cpp
)

ament_target_dependencies(pid_balance_controller
  controller_interface
  pluginlib
  rclcpp
)

# Install the plugin description file
#install(
#  FILES my_custom_controller_plugin.xml
#  DESTINATION share/${PROJECT_NAME}
#)

# Install the header
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS pid_balance_controller
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
