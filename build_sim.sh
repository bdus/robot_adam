#!/bin/bash

cd "$(dirname "$0")"

# 编译 livox_ros_driver2（需要预处理）
cp -f 3rd_packages/livox_ros_driver2/package_ROS2.xml 3rd_packages/livox_ros_driver2/package.xml
cp -rf 3rd_packages/livox_ros_driver2/launch_ROS2/ 3rd_packages/livox_ros_driver2/launch/

colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble 

#rm -rf src/common/livox_ros_driver2/launch/
