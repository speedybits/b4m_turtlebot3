#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Add debug logging
    debug_log = LogInfo(msg='[DEBUG] Starting turtlebot3_webots_nav launch')
    
    # Declare all launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = PathJoinSubstitution([
        FindPackageShare('webots_ros2_turtlebot'),
        'worlds',
        'turtlebot3_burger_example.wbt'
    ])
    robot_name = 'TurtleBot3Burger'  # Must match exactly with the camera topic prefix
    
    # Paths for various packages
    nav2_bringup_path = FindPackageShare('nav2_bringup')
    slam_toolbox_path = FindPackageShare('slam_toolbox')
    webots_turtlebot_path = FindPackageShare('webots_ros2_turtlebot')
    
    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include Webots launch with debug
    webots_debug = LogInfo(msg='[DEBUG] Starting Webots launch')
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
            'world': world,
            'robot_name': robot_name,
            'mode': 'realtime'
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
    
    # Include Nav2 launch with debug
    nav2_debug = LogInfo(msg='[DEBUG] Starting Nav2 launch')
    nav2_launch = TimerAction(
        period=10.0,  # delay in seconds
        actions=[
            nav2_debug,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        nav2_bringup_path,
                        'launch',
                        'navigation_launch.py'
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
        
        # Launch arguments
        declare_use_sim_time,
        
        # Main launches with sequential timing
        webots_debug,
        webots_launch,
        slam_launch,
        nav2_launch,
        b4m_bridge_launch,
        set_initial_pose,
        
        # Log info for user feedback
        LogInfo(msg='[DEBUG] All nodes have been launched. The system should be ready in about 20 seconds.'),
    ])
