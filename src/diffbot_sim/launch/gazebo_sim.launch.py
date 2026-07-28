#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('diffbot_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'empty_arena.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'diffbot.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 alongside Gazebo'
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_diffbot',
        arguments=['-topic', 'robot_description', '-entity', 'diffbot', '-z', '0.05'],
        output='screen'
    )

    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_rviz_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
    ])
