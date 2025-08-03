cmake_minimum_required(VERSION 3.8)
project(wobl_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(
  icm20948
  GIT_REPOSITORY https://github.com/makkhudjakov/SparkFun_ICM-20948_RaspberryLibrary.git
  GIT_TAG main  # or a specific commit/tag
)

FetchContent_MakeAvailable(icm20948)

add_subdirectory(external/scservo)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wobl_messages REQUIRED)

include_directories(include external)

# ============================
# IMU Nodes
# ============================

add_executable(imu_node
  src/imu_node.cpp
  src/imu.cpp
)

add_executable(imu_live_calib
  src/imu_live_calib.cpp
  src/imu.cpp
)

target_link_libraries(imu_node ICM20948_Lib)
target_link_libraries(imu_live_calib ICM20948_Lib)

# Link ROS dependencies once using target dependencies
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(imu_live_calib
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ============================
# Servo Node
# ============================

add_library(servo_driver src/servo_driver.cpp)
target_link_libraries(servo_driver SCServo)

add_executable(servo_test tools/servo_driver_test.cpp)
target_link_libraries(servo_test servo_driver)

add_executable(servo_set_id tools/servo_set_id.cpp)
target_link_libraries(servo_set_id servo_driver)

add_executable(servo_set_midpoint tools/servo_set_midpoint.cpp)
target_link_libraries(servo_set_midpoint servo_driver)

# ============================
# Install Targets
# ============================

install(TARGETS
  imu_node
  imu_live_calib
  servo_test
  servo_set_id
  servo_set_midpoint
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
