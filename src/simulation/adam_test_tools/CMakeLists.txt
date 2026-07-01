cmake_minimum_required(VERSION 3.8)
project(adam_test_tools)

find_package(ament_cmake REQUIRED)

# Script nodes (ros2 run adam_test_tools <name>)
install(PROGRAMS
  scripts/spawn_object.py
  scripts/teleport_robot.py
  scripts/control_light.py
  scripts/spawn_actor.py
  scripts/apply_force.py
  scripts/set_friction.py
  DESTINATION lib/${PROJECT_NAME}
)

# Verification nodes
install(PROGRAMS
  verification/tf_monitor_node.py
  verification/odom_health_node.py
  verification/voxel_decay_meter.py
  verification/pose_jump_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files and worlds
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
