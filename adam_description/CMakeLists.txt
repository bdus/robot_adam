cmake_minimum_required(VERSION 3.8)
project(adam_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)

# livox_ros_driver2 在 3rd_packages 目录
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../3rd_packages/livox_ros_driver2/install/livox_ros_driver2")
find_package(livox_ros_driver2 REQUIRED)

# 包含 Gazebo 头文件和库路径
include_directories(include)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(/usr/include/gazebo-11/gazebo)
link_directories(${GAZEBO_LIBRARY_DIRS})

# 安装目录
install(DIRECTORY
  urdf
  config
  launch
  data
  world
  DESTINATION share/${PROJECT_NAME}/
)

# 编译 ros2_livox 库 (Mid360 激光雷达 Gazebo 插件)
add_library(ros2_livox SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(ros2_livox ${GAZEBO_LIBRARIES} RayPlugin GpuRayPlugin)
ament_target_dependencies(ros2_livox rclcpp std_msgs sensor_msgs geometry_msgs gazebo_dev gazebo_ros tf2_ros livox_ros_driver2)
target_link_libraries(ros2_livox libprotobuf.so.9)
target_link_libraries(ros2_livox libboost_chrono.so)
target_include_directories(ros2_livox PRIVATE include)

# 安装 ros2_livox 库
install(TARGETS ros2_livox
  LIBRARY DESTINATION lib
)

ament_export_libraries(
  ros2_livox
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
