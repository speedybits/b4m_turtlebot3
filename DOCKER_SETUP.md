# Docker Setup Instructions

This guide explains how to set up and run the ROS2 Turtlebot3 environment using Docker.

## Prerequisites

### For Linux:
```bash
# Install Docker
sudo apt-get update
sudo apt-get remove docker docker-engine docker.io containerd runc
sudo apt-get update && sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg && sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update && sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add your user to the docker group (logout and login required after this)
sudo usermod -aG docker $USER

# Install X11 requirements (if not already installed)
sudo apt-get install x11-xserver-utils
```

### For Mac (Apple Silicon):
1. Install Docker Desktop for Mac from https://www.docker.com/products/docker-desktop/
2. Install XQuartz:
```bash
brew install --cask xquartz
```

## Building and Running

1. Clone the repository:
```bash
git clone git@github.com:speedybits/b4m_turtlebot3.git
cd b4m_turtlebot3
# Switch to the b4m_integration branch
git checkout b4m_integration
```

2. Build the Docker image:
```bash
docker compose build
```

3. Start the container:
```bash
docker compose up -d
```

4. Enter the container:
```bash
docker compose exec ros2_dev bash
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

All ROS2 commands should be run inside the container. To launch Webots with the optimized settings:

```bash
# Inside the container
./b4m_launch.sh
```

The `b4m_launch.sh` script includes:
- Performance optimization settings for Webots
- Minimized debug output
- Proper ROS2 logging configuration
- Automatic sourcing of ROS2 environment
- Launch of TurtleBot3 with navigation enabled

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
