# Docker Setup for B4M ROS2 Development

This document describes how to set up and use the Docker development environment for the B4M ROS2 project.

## Prerequisites

- Docker installed on your system
- Docker Compose installed on your system
- Git with SSH access configured

## Project Structure

The Docker environment is configured to build and run the following ROS2 packages:
- `b4m_bridge`: ROS2 bridge node for processing sensor data
- `b4m_voice`: Voice control package
- `b4m_camera`: Camera handling package

## Building the Environment

1. Clone the repository:
```bash
git clone git@github.com:speedybits/b4m_turtlebot3.git
cd b4m_turtlebot3
# Switch to the b4m_integration branch
git checkout b4m_integration
```

2. Build and start the Docker container:
```bash
docker compose build
docker compose up -d
```

The build process will:
- Use ROS2 Humble as the base image
- Install all necessary ROS2 dependencies
- Install Python packages including `bike4py`
- Build the workspace using colcon

### Using the Environment

#### Entering the Container

To enter the Docker container:
```bash
docker compose exec ros2_dev bash
```

#### Running ROS2 Commands

Once inside the container, you can run ROS2 commands. The environment is already set up with:
- ROS2 Humble sourced
- Workspace packages built and sourced
- All dependencies installed

Common commands:
```bash
# List available packages
ros2 pkg list | grep b4m

# List available topics
ros2 topic list

# Run nodes from b4m_bridge
ros2 run b4m_bridge <node_name>

# Run nodes from b4m_voice
ros2 run b4m_voice <node_name>
```

#### Development Workflow

1. Make changes to the code on your host machine
2. Rebuild the workspace in the container:
```bash
docker compose exec ros2_dev bash -c "cd /workspace && colcon build"
```
3. Source the updated workspace:
```bash
source /workspace/install/setup.bash
```

### Container Configuration

#### Dockerfile

The Dockerfile includes:
- ROS2 Humble base image
- Essential ROS2 packages:
  - geometry_msgs
  - nav_msgs
  - sensor_msgs
  - cv_bridge
  - image_transport
- Python development tools
- Custom entrypoint script for environment setup

#### Docker Compose

The docker-compose.yml configuration:
- Mounts the workspace directory
- Sets ROS_DOMAIN_ID for ROS2 communication
- Uses host network mode for easier ROS2 communication

### Troubleshooting

If you encounter any issues:

1. Ensure all dependencies are installed:
```bash
docker compose exec ros2_dev bash -c "ros2 pkg list"
```

2. Check the workspace build status:
```bash
docker compose exec ros2_dev bash -c "source /opt/ros/humble/setup.bash && colcon build --event-handlers console_direct+"
```

3. Verify environment variables:
```bash
docker compose exec ros2_dev env | grep ROS
```

### Cleaning Up

To stop and remove the container:
```bash
docker compose down
```

To remove all built images and start fresh:
```bash
docker compose down --rmi all
```

## X11 Setup for GUI Applications

### Linux:
No additional setup needed. GUI applications should work out of the box.

### Mac:
1. Start XQuartz
2. In XQuartz preferences:
   - Go to Security tab
   - Check "Allow connections from network clients"
3. Restart XQuartz
4. In terminal:
```bash
xhost +localhost
```

## Development Workflow

1. Edit code on your host machine using your preferred editor
2. Files are automatically synced with the container via the mounted volume
3. Build and run code inside the container
4. Use git commands on your host machine

## Common Commands

```bash
# Start the container
docker compose up -d

# Enter the container
docker compose exec ros2_dev bash

# View container logs
docker compose logs

# Stop the container
docker compose down

# Rebuild the container (after changes to Dockerfile)
docker compose build --no-cache
```

## Running ROS2 Commands

All ROS2 commands should be run inside the container.

## Troubleshooting

1. If GUI applications don't work:
   - Check X11 forwarding setup
   - Ensure DISPLAY variable is set correctly
   - Try restarting XQuartz (Mac) or X server (Linux)
   - Make sure Webots is not already running in another container or on the host

2. If the container fails to start:
   - Check Docker logs: `docker compose logs`
   - Ensure no other containers are using the same ports
   - Verify all required devices are available

3. If changes to source code don't take effect:
   - Rebuild the workspace: `colcon build`
   - Source the setup file: `source install/setup.bash`

4. If Webots performance is slow:
   - The script already includes performance optimizations
   - Check your system's GPU/CPU usage
   - Consider reducing other system load
