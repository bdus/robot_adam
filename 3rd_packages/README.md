# Third-Party Dependencies for Robot Adam

This directory contains third-party dependencies that are built from source and integrated into the workspace.

## Dependencies

1. **Sophus** - Lie group library for state estimation
   - Source: https://github.com/strasdat/Sophus
   - Version: 1.22.10 (with basic logging)
   - Build instructions: See FASTLIO2_ROS2 README.md

2. **FAST-LIO2** - 3D LiDAR-Inertial Odometry
   - This is a symlink or copy of the FASTLIO2_ROS2 repository
   - Contains: fastlio2/, hba/, localizer/, pgo/, interface/ packages

3. **Livox ROS2 Driver** - Official driver for Livox Mid360 and other LiDARs
   - Source: https://github.com/Livox-SDK/livox_ros_driver2

## Build Order

1. Install Livox-SDK2 (system dependency)
2. Build livox_ros_driver2
3. Build Sophus (if needed by FAST-LIO2)
4. Build FASTLIO2_ROS2 workspace

Each dependency should have its own build instructions in their respective directories.