#!/usr/bin/env python3
"""
teleop.launch.py

Launches teleop_twist_keyboard in its own terminal-friendly node.
This publishes geometry_msgs/Twist on /cmd_vel, which is consumed by:
  - Gazebo's diff_drive plugin (simulated robot moves on screen)
  - serial_bridge_node.py (real Arduino car moves, once wired up)

Run this in a SEPARATE terminal from gazebo_sim.launch.py, because
teleop_twist_keyboard needs an interactive terminal with keyboard focus.

  ros2 launch diffbot_sim teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # opens in its own terminal window so key capture works
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([teleop_node])
