from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='Use simulated speech input (true) or real microphone (false)'
    )
    
    # Create the node
    voice_control_node = Node(
        package='b4m_voice',
        executable='voice_control',
        name='voice_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_simulation': LaunchConfiguration('use_simulation')
        }]
    )
    
    return LaunchDescription([
        use_simulation_arg,
        voice_control_node
    ])
