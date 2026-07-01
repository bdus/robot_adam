cmake_minimum_required(VERSION 3.8)
project(adam_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/archive_map.py DESTINATION lib/${PROJECT_NAME})

ament_package()