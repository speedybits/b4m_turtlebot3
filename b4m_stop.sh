#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash

# Function to ensure a process is killed
ensure_killed() {
    local pattern="$1"
    local max_attempts=3
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        pids=$(pgrep -f "$pattern" || true)
        if [ -z "$pids" ]; then
            return 0
        fi
        
        echo "Attempt $attempt to kill processes matching '$pattern' (PIDs: $pids)..."
        if [ $attempt -eq $max_attempts ]; then
            echo "Using SIGKILL as last resort..."
            for pid in $pids; do
                kill -9 $pid 2>/dev/null || true
            done
        else
            for pid in $pids; do
                kill $pid 2>/dev/null || true
            done
        fi
        
        sleep 2
        attempt=$((attempt + 1))
    done
    
    # Check if any processes still remain
    pids=$(pgrep -f "$pattern" || true)
    if [ ! -z "$pids" ]; then
        echo "Warning: Failed to kill all processes matching '$pattern' (PIDs: $pids)"
        return 1
    fi
    return 0
}

echo "Stopping ROS2 processes..."

# Show current ROS2 processes
echo "Current ROS2 processes:"
ps aux | grep -E "ros2|/opt/ros/humble/lib/" | grep -v grep || true

# Kill specific ROS2 nodes that we commonly use
NODES_TO_KILL=(
    "static_transform_publisher"
    "robot_state_publisher"
    "tf2_echo"
    "smoother_server"
    "planner_server"
    "behavior_server"
    "bt_navigator"
    "waypoint_follower"
    "velocity_smoother"
    "lifecycle_manager"
    "async_slam_toolbox"
    "component_container"
    "b4m_bridge"           # Added b4m_bridge
    "b4m_voice"            # Added b4m_voice
    "b4m_camera"           # Added b4m_camera
)

# Stop each node
for node in "${NODES_TO_KILL[@]}"; do
    ensure_killed "$node"
done

echo "Stopping ROS2 daemon..."
timeout 10s ros2 daemon stop

# Final cleanup of any remaining ROS2 processes
echo "Final cleanup of any remaining processes..."
ensure_killed "ros2"
ensure_killed "/opt/ros/humble/lib/"
ensure_killed "webots"

# Verify all processes are stopped
echo "Verifying all processes are stopped..."
remaining=$(ps aux | grep -E "ros2|/opt/ros/humble/lib/" | grep -v grep || true)
if [ ! -z "$remaining" ]; then
    echo "Warning: Some processes are still running:"
    echo "$remaining"
    exit 1
else
    echo "All processes successfully stopped."
    exit 0
fi
