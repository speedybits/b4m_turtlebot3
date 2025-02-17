#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Add debug logging
    debug_log = LogInfo(msg='[DEBUG] Starting b4m_webots_launch')
    
    # Declare all launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get the path to our custom world file using FindPackageShare
    b4m_bridge_path = FindPackageShare('b4m_bridge')
    world = PathJoinSubstitution([b4m_bridge_path, 'worlds', 'b4m_world.wbt'])
    
    # Log the world file path for debugging
    world_debug = LogInfo(msg=f'[DEBUG] Using world file from package share')
    
    robot_name = 'TurtleBot3Burger'  # Must match exactly with the camera topic prefix
    
    # Paths for various packages
    slam_toolbox_path = FindPackageShare('slam_toolbox')
    webots_turtlebot_path = FindPackageShare('webots_ros2_turtlebot')
    b4m_camera_path = FindPackageShare('b4m_camera')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include Webots launch with navigation enabled
    webots_debug = LogInfo(msg='[DEBUG] Starting Webots launch with navigation')
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                webots_turtlebot_path,
                'launch',
                'robot_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_description_overlay': PathJoinSubstitution([b4m_camera_path, 'urdf', 'b4m_camera.urdf']),
            'world': world
        }.items()
    )
    
    # Include SLAM launch with debug
    slam_debug = LogInfo(msg='[DEBUG] Starting SLAM launch')
    slam_launch = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            slam_debug,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        slam_toolbox_path,
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # Launch b4m_bridge with debug
    b4m_debug = LogInfo(msg='[DEBUG] Starting b4m_bridge launch')
    b4m_bridge_launch = TimerAction(
        period=15.0,  # delay in seconds
        actions=[
            b4m_debug,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('b4m_bridge'),
                        'launch',
                        'b4m_bridge.launch.py'
                    ])
                ])
            )
        ]
    )
    
    # Set initial pose with delay
    set_initial_pose = TimerAction(
        period=20.0,  # delay in seconds
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='initial_pose_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        # Debug logging
        debug_log,
        
        # Log the world file path for debugging
        world_debug,
        
        # Launch arguments
        declare_use_sim_time,
        
        # Main launches with sequential timing
        webots_debug,
        webots_launch,  # This includes Nav2 and RViz2
        slam_launch,
        b4m_bridge_launch,
        set_initial_pose,
        
        # Log info for user feedback
        LogInfo(msg='[DEBUG] All nodes have been launched. The system should be ready in about 20 seconds.'),
    ])
