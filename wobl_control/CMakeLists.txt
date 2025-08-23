
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
add_executable(wobl_controller
  src/wobl_controller_node.cpp
  src/pid_balance_controller.cpp
  src/wobl_state.cpp
  src/wobl_command.cpp
  src/config.cpp
)
target_include_directories(wobl_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(wobl_controller PRIVATE ${WOBL_DEPS})

add_executable(wheel_eval_node src/wheel_eval_node.cpp)
target_include_directories(wheel_eval_node PRIVATE include)
target_link_libraries(wheel_eval_node PRIVATE ${WOBL_DEPS})

install(TARGETS
  wobl_controller
  wheel_eval_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
