from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    debug_log = LogInfo(msg='[DEBUG] Starting b4m_bridge.launch.py')
    bridge_debug = LogInfo(msg='[DEBUG] Launching b4m_bridge node')
    camera_debug = LogInfo(msg='[DEBUG] Launching b4m_camera node')

    return LaunchDescription([
        debug_log,
        
        # Launch main b4m_bridge node
        bridge_debug,
        Node(
            package='b4m_bridge',
            executable='b4m_bridge',
            name='b4m_bridge',
            output='screen',
            emulate_tty=True,
        ),

        # Launch camera node
        camera_debug,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('b4m_camera'),
                    'launch',
                    'b4m_camera.launch.py'
                ])
            ])
        )
    ])
