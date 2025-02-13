#!/bin/bash

# Performance optimization settings
export WEBOTS_DISABLE_SHADOWS=1
export WEBOTS_PREFER_SIMPLE_TEXTURES=1
export WEBOTS_FAST_TEXTURE_FILTERING=1
export WEBOTS_DISABLE_ANTI_ALIASING=1
export WEBOTS_CONFIG_FILE="$(pwd)/b4m_webots_config.json"

# Minimize debug output
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_COLORIZED_OUTPUT=0
export WEBOTS_STDOUT_REDIRECT=0
export WEBOTS_DEBUG_LEVEL=0

# Set ROS logging to only show warnings and errors
export RCUTILS_CONSOLE_LEVEL=30

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Turtlebot3 with navigation enabled
ros2 launch b4m_bridge b4m_webots_launch.py