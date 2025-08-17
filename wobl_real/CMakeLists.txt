cmake_minimum_required(VERSION 3.8)
project(wobl_real)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(wobl_msgs REQUIRED)

# External libraries
include(FetchContent)
FetchContent_Declare(
  icm20948
  GIT_REPOSITORY https://github.com/makkhudjakov/SparkFun_ICM-20948_RaspberryLibrary.git
  #GIT_TAG main
  GIT_TAG 807f55b4b79a45bc064839fb67a0ffb3bb6e3377
)
FetchContent_MakeAvailable(icm20948)
add_subdirectory(external/scservo)

# Common dependencies
set(WOBL_DEPS
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${wobl_msgs_TARGETS}
)

# IMU library and nodes
add_library(libimu src/imu.cpp)
target_include_directories(libimu PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external
)
target_link_libraries(libimu PUBLIC ICM20948_Lib ${WOBL_DEPS})

add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node PUBLIC libimu)

add_executable(imu_live_calib tools/imu_live_calib.cpp)
target_link_libraries(imu_live_calib PUBLIC libimu)

# Servo library and nodes
add_library(libservo src/servo_driver.cpp)
target_include_directories(libservo PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external
)
target_link_libraries(libservo PUBLIC SCServo)

add_executable(servo_node src/servo_node.cpp)
target_link_libraries(servo_node PUBLIC libservo ${WOBL_DEPS})

add_executable(servo_test tools/servo_driver_test.cpp)
target_link_libraries(servo_test PUBLIC libservo rclcpp::rclcpp)

add_executable(servo_set_id tools/servo_set_id.cpp)
target_link_libraries(servo_set_id PUBLIC libservo rclcpp::rclcpp)

add_executable(servo_set_midpoint tools/servo_set_midpoint.cpp)
target_link_libraries(servo_set_midpoint PUBLIC libservo rclcpp::rclcpp)

add_executable(servo_ping tools/servo_ping.cpp)
target_link_libraries(servo_ping PUBLIC libservo rclcpp::rclcpp)

add_executable(robot_test tools/robot_test.cpp)
target_link_libraries(robot_test PUBLIC ${WOBL_DEPS})

# Install targets
install(TARGETS
  imu_node
  servo_node
  imu_live_calib
  servo_test
  servo_set_id
  servo_set_midpoint
  robot_test
  servo_ping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()