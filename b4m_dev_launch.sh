#!/bin/bash

# Function to print section headers
print_header() {
    echo "========================================="
    echo "$1"
    echo "========================================="
}

# Function to check if container is running
check_container() {
    if ! docker compose ps | grep -q "ros2_dev.*running"; then
        print_header "Starting Docker container..."
        docker compose up -d
        sleep 2  # Give container time to start
    fi
}

# Function to build packages
build_packages() {
    print_header "Building ROS2 packages..."
    docker compose exec ros2_dev bash -c "cd /workspace && \
        source /opt/ros/humble/setup.bash && \
        colcon build --packages-select b4m_bridge b4m_voice"
}

# Main script
print_header "B4M Development Environment Setup"

# Check and start container if needed
check_container

# Build packages
build_packages

# Launch instructions
print_header "Launch Instructions"
echo "Open two terminal windows and run these commands in each:"
echo
echo "Terminal 1 (Bridge Node):"
echo "docker compose exec ros2_dev bash"
echo "source /opt/ros/humble/setup.bash"
echo "source /workspace/install/setup.bash"
echo "ros2 run b4m_bridge b4m_bridge"
echo
echo "Terminal 2 (Voice Node):"
echo "docker compose exec ros2_dev bash"
echo "source /opt/ros/humble/setup.bash"
echo "source /workspace/install/setup.bash"
echo "ros2 run b4m_voice voice_control"
