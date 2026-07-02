cmake_minimum_required(VERSION 3.8)
project(custom_explorer)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(explorer src/explorer.cpp)
ament_target_dependencies(explorer rclcpp)
install(TARGETS explorer
  DESTINATION lib/${PROJECT_NAME})
ament_package()
