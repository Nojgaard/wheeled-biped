cmake_minimum_required(VERSION 3.8)
project(wobl_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  mjcf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
