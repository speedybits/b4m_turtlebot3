from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Start the voice control node
    voice_control_node = Node(
        package='b4m_voice',
        executable='voice_control',
        name='voice_control_node',
        output='screen',
        parameters=[{'use_simulation': True}]
    )
    
    # Get the path to the test file
    pkg_dir = get_package_share_directory('b4m_voice')
    test_file = os.path.join(pkg_dir, 'test', 'b4m_voice_test.py')
    
    # Start the test
    test_process = ExecuteProcess(
        cmd=['python3', test_file],
        output='screen'
    )
    
    return LaunchDescription([
        voice_control_node,
        test_process
    ])
