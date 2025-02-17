#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='b4m_camera',
            executable='b4m_camera_node',
            name='b4m_camera',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
