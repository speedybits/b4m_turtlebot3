# B4M TurtleBot3 Project

This repository contains the ROS2 packages for the B4M TurtleBot3 project, which includes:
- `b4m_bridge`: ROS2 bridge node for processing sensor data and controlling the robot
- `b4m_voice`: Voice control package for natural language interaction
- `b4m_camera`: Camera handling package

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

3. Enter the container and build the workspace:
```bash
docker compose exec ros2_dev bash
colcon build
source install/setup.bash
```

## Docker Environment Setup

### Prerequisites
- Docker installed on your system
- Docker Compose installed on your system
- Git with SSH access configured

### Container Configuration
The Docker environment includes:
- ROS2 Humble base image
- Essential ROS2 packages (geometry_msgs, nav_msgs, sensor_msgs, cv_bridge, image_transport)
- Python development tools
- bike4py package for B4M integration

### Development Workflow
1. Make changes to the code on your host machine
2. Rebuild the workspace in the container:
```bash
docker compose exec ros2_dev bash -c "cd /workspace && colcon build"
```
3. Source the updated workspace:
```bash
source /workspace/install/setup.bash
```

## Testing Voice Control

To test the voice interaction between `b4m_bridge` and `b4m_voice`, follow these steps:

1. Make sure you have a valid B4M API token in `b4m_api_token.txt` at the workspace root.

2. Open three terminal windows and enter the Docker container in each:
```bash
docker compose exec ros2_dev bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

3. In the first terminal, start the b4m_bridge node:
```bash
ros2 run b4m_bridge b4m_bridge_node
```

4. In the second terminal, start the voice control node in simulation mode:
```bash
ros2 run b4m_voice voice_control_node --ros-args -p use_simulation:=true
```

5. In the third terminal, simulate speech input:
```bash
ros2 topic pub /speaker/speech_input std_msgs/msg/String "data: 'Hello robot, tell me something interesting'"
```

The robot should process your request through the voice control node, send it to the B4M bridge, and respond with an interesting fact or statement.

### Troubleshooting

1. Check if nodes are running:
```bash
ros2 node list
```

2. Monitor speech processing:
```bash
ros2 topic echo /speech_text
ros2 topic echo /speech_status
```

3. Check B4M bridge status:
```bash
ros2 topic echo /b4m_status
```

For more detailed setup instructions, please refer to [DOCKER_SETUP.md](DOCKER_SETUP.md).

# To run the simulation
./b4m_launch.sh
