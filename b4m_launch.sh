#!/bin/bash

# Source ROS 2 environment
#source /opt/ros/humble/setup.bash

# Launch Turtlebot3 with navigation enabled
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true

# Similar launch file is here (ignore for now):
# /src/b4m_bridge/launch/turtlebot3_webots_nav.launch.py