#!/bin/bash

# Function to print section headers
print_header() {
    echo "========================================="
    echo "$1"
    echo "========================================="
}

# Function to check if Docker is running and accessible
check_docker() {
    if ! docker info >/dev/null 2>&1; then
        print_header "Docker Not Running"
        if [[ "$(uname)" == "Darwin" ]]; then
            echo "Mac Users: Please start Docker Desktop from your Applications or Launchpad"
            echo "1. Open Launchpad"
            echo "2. Click on Docker Desktop"
            echo "3. Wait for Docker to finish starting"
            echo "4. Try this command again"
        else
            echo "Linux Users: Please start the Docker daemon:"
            echo "sudo systemctl start docker"
            echo "# Or check status:"
            echo "sudo systemctl status docker"
        fi
        exit 1
    fi
}

# Function to check if container is running
check_container() {
    # First check if Docker is running
    check_docker
    
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

# Function to run the bridge node
run_bridge() {
    docker compose exec ros2_dev bash -c "\
        source /opt/ros/humble/setup.bash && \
        source /workspace/install/setup.bash && \
        ros2 run b4m_bridge b4m_bridge"
}

# Function to run the voice control node
run_voice() {
    docker compose exec ros2_dev bash -c "\
        source /opt/ros/humble/setup.bash && \
        source /workspace/install/setup.bash && \
        ros2 run b4m_voice voice_control"
}

# Function to send a simulated voice command
send_command() {
    local command="$1"
    docker compose exec ros2_dev bash -c "\
        source /opt/ros/humble/setup.bash && \
        source /workspace/install/setup.bash && \
        ros2 topic pub --once /speaker/speech_input std_msgs/msg/String \"data: '$command'\""
}

# Function to print usage instructions
print_usage() {
    echo "Usage:"
    echo "  ./b4m_dev_launch.sh                    # Setup environment"
    echo "  ./b4m_dev_launch.sh bridge            # Run bridge node"
    echo "  ./b4m_dev_launch.sh voice             # Run voice control node"
    echo "  ./b4m_dev_launch.sh send \"<command>\"   # Send voice command"
    echo
    echo "Available voice commands:"
    echo "  - \"move forward\"  - Start moving forward"
    echo "  - \"stop\"          - Stop movement"
    echo "  - \"turn left\"     - Rotate left"
    echo "  - \"turn right\"    - Rotate right"
    echo "  - \"status\"        - Get robot status"
}

# Main script
case "$1" in
    "bridge")
        run_bridge
        ;;
    "voice")
        run_voice
        ;;
    "send")
        if [ -z "$2" ]; then
            echo "Error: Please provide a command to send"
            print_usage
            exit 1
        fi
        send_command "$2"
        ;;
    "help")
        print_usage
        ;;
    *)
        print_header "B4M Development Environment Setup"
        check_container
        build_packages

        print_header "Launch Instructions"
        echo "1. Start the nodes in separate terminals:"
        echo
        echo "Terminal 1 (Bridge Node):"
        echo "./b4m_dev_launch.sh bridge"
        echo
        echo "Terminal 2 (Voice Node):"
        echo "./b4m_dev_launch.sh voice"
        echo
        echo "2. Send voice commands from any terminal:"
        echo "./b4m_dev_launch.sh send \"move forward\""
        echo "./b4m_dev_launch.sh send \"stop\""
        echo
        echo "For more commands and help:"
        echo "./b4m_dev_launch.sh help"
        ;;
esac
