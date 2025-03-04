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

## Development Guide

### Accessing the Container
You can access the Docker container in two ways:

1. Open a new shell in the container:
```bash
docker compose exec ros2_dev bash
```

2. Run a one-off command in the container:
```bash
docker compose exec ros2_dev bash -c "your_command_here"
```

Remember to source the workspace when entering a new shell:
```bash
source /workspace/install/setup.bash
```

To open multiple terminals in the same container, run the first command in each new terminal window.

### Code Organization
- `b4m_bridge/`: ROS2 bridge node for processing sensor data and controlling the robot
  - `src/`: C++ source files
  - `include/`: Header files
  - `scripts/`: Python utilities
- `b4m_voice/`: Voice control package
  - `src/`: Python source files
  - `config/`: Configuration files for voice recognition

### Making Code Changes
1. Edit source files directly on your host machine using your preferred editor
   - All changes are immediately reflected in the container due to volume mounting
   - Use Git on your host machine for version control

2. Rebuild after making changes:
```bash
# Rebuild specific packages
docker compose exec ros2_dev bash -c "cd /workspace && colcon build --packages-select b4m_bridge b4m_voice"

# Or rebuild all packages
docker compose exec ros2_dev bash -c "cd /workspace && colcon build"
```

3. Source the workspace after rebuilding:
```bash
source /workspace/install/setup.bash
```

### Development Tips
- Use `--symlink-install` with colcon build to avoid rebuilding for Python changes:
```bash
colcon build --symlink-install --packages-select b4m_voice
```
- For C++ changes in `b4m_bridge`, always do a full rebuild of the package
- Use `colcon build --packages-up-to <package>` to rebuild a package and its dependencies
- Run tests for your changes:
```bash
colcon test --packages-select b4m_bridge b4m_voice
```

### Common Issues
- If changes aren't taking effect, ensure you've:
  1. Rebuilt the affected packages
  2. Sourced the workspace
  3. Restarted any running nodes
- For build errors, check:
  1. All dependencies are properly listed in `package.xml`
  2. CMake configuration in `CMakeLists.txt`
  3. Build output with `--event-handlers console_direct+`

For detailed Docker setup and configuration, see [DOCKER_SETUP.md](DOCKER_SETUP.md).

## Development Workflow
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

1. Enter the container and source the workspace:
```bash
docker compose exec ros2_dev bash
source /workspace/install/setup.bash
```

2. Run the bridge node:
```bash
ros2 run b4m_bridge bridge_node
```

3. Run the voice control node (in a new terminal):
```bash
# In a new terminal, enter the container again
docker compose exec ros2_dev bash
source /workspace/install/setup.bash

# Run the voice node
ros2 run b4m_voice voice_node
```

### Testing Voice Control with Bridge Node
1. Start the bridge node in one terminal:
```bash
ros2 run b4m_bridge bridge_node
```

2. In another terminal, start the voice node:
```bash
ros2 run b4m_voice voice_node
```

3. Verify the nodes are communicating:
```bash
# Check if both nodes are running
ros2 node list

# View available topics
ros2 topic list

# Monitor voice commands being processed
ros2 topic echo /voice_commands

# Monitor bridge node status
ros2 topic echo /bridge_status
```

4. Test basic voice commands:
   - Say "move forward" to initiate forward movement
   - Say "stop" to halt the robot
   - Say "turn left" or "turn right" for rotation
   - Say "status" to get the current robot state

Note: Ensure both nodes are running before testing voice commands. The bridge node must be active to process the voice commands and control the robot.

#### Using Text Input Instead of Microphone
You can send text commands directly to test the voice control system without using a microphone:

```bash
# Send a text command
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'move forward'"

# Other example commands:
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'stop'"
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'turn left'"
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'status'"
```

The voice node processes text commands the same way as voice input, making this method useful for:
- Testing without a microphone
- Debugging voice command processing
- Automating command sequences
- CI/CD testing

## Notes
- This setup focuses on `b4m_bridge` and `b4m_voice` packages for a minimal development environment
- The Docker container mounts the workspace directory, so you can edit code on your host machine
- All changes are immediately reflected in the container
- The container uses network_mode: host for easy ROS2 communication with the host
