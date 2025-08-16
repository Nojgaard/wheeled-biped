
cmake_minimum_required(VERSION 3.8)
project(wobl_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(wobl_msgs REQUIRED)

# Common dependencies
set(WOBL_DEPS
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${control_toolbox_TARGETS}
  ${geometry_msgs_TARGETS}
  ${wobl_msgs_TARGETS}
)

# Executable
add_executable(pid_balance_controller src/pid_balance_controller_node.cpp)
target_include_directories(pid_balance_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pid_balance_controller PRIVATE ${WOBL_DEPS})

install(TARGETS
  pid_balance_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
