#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source the workspace if it exists
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Add ROS2 environment to .bashrc for interactive shells
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
if [ -f "/workspace/install/setup.bash" ]; then
    echo "source /workspace/install/setup.bash" >> /root/.bashrc
fi

# Execute the command passed to docker run
exec "$@"
