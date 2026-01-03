#!/usr/bin/env python3
"""
Launch file pour le scénario de test simple (vitre unique)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Chemins
    pkg_share = FindPackageShare('windowclean_pro')
    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'simple_window.world'
    ])
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'windowclean_pro.urdf.xacro'
    ])
    
    # Arguments de lancement
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    world = LaunchConfiguration('world', default=world_file)
    
    # Robot state publisher avec URDF généré via xacro
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', urdf_file])}
        ]
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world],
        output='screen'
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'windowclean_pro',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.6',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to world model file'
        ),
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])

