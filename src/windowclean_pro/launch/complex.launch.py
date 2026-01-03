#!/usr/bin/env python3
"""
Launch file pour le scénario complexe avec obstacles
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('windowclean_pro')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'complex.world'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'windowclean_pro.urdf.xacro'])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ExecuteProcess(
                cmd=['xacro', urdf_file],
                output='screen'
            )
        }],
        output='screen'
    )
    
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'windowclean_pro',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '2.6'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])

