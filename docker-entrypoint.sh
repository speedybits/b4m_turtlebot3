#!/bin/bash

# Source ROS environment
source /opt/ros/humble/setup.bash

# Source the workspace if it exists
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"
