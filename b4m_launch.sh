#!/bin/bash

# Source ROS 2 environment
#source /opt/ros/humble/setup.bash

# Launch Turtlebot3 with navigation enabled
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
