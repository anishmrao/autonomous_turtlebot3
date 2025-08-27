cmake_minimum_required(VERSION 3.8)
project(tb3_bringup)

find_package(ament_cmake REQUIRED)

# We only install resources (no C++ code)
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()