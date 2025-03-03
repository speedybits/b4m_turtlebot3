# B4M TurtleBot3 Project

This repository contains the ROS2 packages for the B4M TurtleBot3 project, focusing on:
- `b4m_bridge`: ROS2 bridge node for processing sensor data and controlling the robot
- `b4m_voice`: Voice control package for natural language interaction

## Quick Start

1. Clone the repository:
```bash
git clone git@github.com:speedybits/b4m_turtlebot3.git
cd b4m_turtlebot3
```

2. Start the Docker environment:
```bash
docker compose up -d
```

3. Build and run the workspace:
```bash
# Build just b4m_bridge and b4m_voice
docker compose exec ros2_dev bash -c "cd /workspace && colcon build --packages-select b4m_bridge b4m_voice"

# Enter the container to run nodes
docker compose exec ros2_dev bash
source install/setup.bash
```

## Docker Environment Setup

### Prerequisites
- Docker installed on your system
- Docker Compose installed on your system
- Git with SSH access configured

### Container Configuration
The Docker environment is kept minimal and includes:
- ROS2 Humble base image
- Essential ROS2 message packages (geometry_msgs, nav_msgs, sensor_msgs)
- Python development tools (pip, colcon, pytest, flake8)
- bike4py package for B4M integration

### Development Workflow
1. Make changes to the code on your host machine
2. Rebuild specific packages in the container:
```bash
# Rebuild just b4m_bridge and b4m_voice
docker compose exec ros2_dev bash -c "cd /workspace && colcon build --packages-select b4m_bridge b4m_voice"
```
3. Source the updated workspace:
```bash
source /workspace/install/setup.bash
```

### Running the Nodes
From inside the container:

1. Run the bridge node:
```bash
ros2 run b4m_bridge bridge_node
```

2. Run the voice control node:
```bash
ros2 run b4m_voice voice_node
```

## Notes
- This setup focuses on `b4m_bridge` and `b4m_voice` packages for a minimal development environment
- The Docker container mounts the workspace directory, so you can edit code on your host machine
- All changes are immediately reflected in the container
- The container uses network_mode: host for easy ROS2 communication with the host
