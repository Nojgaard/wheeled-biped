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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ============================
# Common include dirs
# ============================

set(PROJECT_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external
)

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

target_include_directories(imu_node PUBLIC ${PROJECT_INCLUDE_DIRS})
target_include_directories(imu_live_calib PUBLIC ${PROJECT_INCLUDE_DIRS})

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

add_executable(servo_node
  src/servo_driver.cpp
)

target_include_directories(servo_node PUBLIC ${PROJECT_INCLUDE_DIRS})

# If your scservo lib builds a library, link it:
# Example:
# target_link_libraries(servo_node PUBLIC scservo_lib)

ament_target_dependencies(servo_node
  rclcpp
)

# ============================
# Install Targets
# ============================

install(TARGETS
  imu_node
  imu_live_calib
  servo_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
