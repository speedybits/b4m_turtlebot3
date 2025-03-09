# B4M TurtleBot3 Project

This repository contains the ROS2 packages for the B4M TurtleBot3 project, focusing on:
- `b4m_bridge`: ROS2 bridge node for processing sensor data and controlling the robot
- `b4m_voice`: Voice control package for natural language interaction

## Prerequisites

1. Docker installed and configured (see [DOCKER_SETUP.md](DOCKER_SETUP.md))
2. B4M API token (required for robot communication)
3. Working microphone (for voice control)

### Setting up B4M API Token

It'll want an API token. The current place to get that is from Chrome (or your favorite browser). Right-click and go to Inspect, then to the Application tab, then to Local Storage, then be sure you've selected https://app.bike4mind.com/. One of the keys in local storage for the site will be `access-token-storage`, and it'll hold a bit of JSON. One of the JSON keys is `refresh_token` - that's the value it will want.

The token will unfortunately only last a day or two, it seems, and then you'll want to retrieve a new token from the same spot.

Create a file named `b4m_api_token.txt` in the project root with your B4M refresh token. You can do this in one of two ways:

1. Using a text editor (recommended):
   - Open a new file named `b4m_api_token.txt` in your preferred text editor
   - Paste your refresh token exactly as it appears in the browser
   - Save the file

2. Using the terminal:
   ```bash
   cat << 'EOF' > b4m_api_token.txt
   your_b4m_refresh_token
   EOF
   ```
   Replace `your_b4m_refresh_token` with your actual token. The single quotes around EOF will preserve any special characters in the token.

This token is required for the bridge node to communicate with the robot. Contact your system administrator if you need a token.

### Voice Control Dependencies

The voice control node requires several dependencies that are automatically installed in the Docker image:
- Python SpeechRecognition library
- PyAudio for microphone input
- PortAudio system library

### Voice Control Setup

The voice control system has different capabilities depending on your operating system:

#### macOS Users
On macOS, voice control is available in simulation mode only:

```bash
# Launch with simulation mode (recommended for macOS)
ros2 launch b4m_voice b4m_voice.launch.py
```

#### Ubuntu Linux Users
On Ubuntu, you can use both simulation mode and real microphone input:

1. Setup audio permissions:
```bash
sudo usermod -a -G audio $USER
# Log out and log back in for changes to take effect
```

2. Choose your launch mode:
```bash
# Launch with simulation mode
ros2 launch b4m_voice b4m_voice.launch.py

# Launch with real microphone (Ubuntu only)
ros2 launch b4m_voice b4m_voice.launch.py use_simulation:=false
```

#### Voice Commands
Available voice commands in both simulation and real microphone modes:
- "move forward" - Start moving forward
- "stop" - Stop movement
- "turn left/right" - Rotate in place
- "status" - Get robot status

#### Troubleshooting
- For simulation mode issues, check the ROS2 logs
- For microphone issues on Ubuntu, see [DOCKER_SETUP.md](DOCKER_SETUP.md)
- MacOS users should use simulation mode only

## Why Docker?

This project uses Docker to ensure a consistent development environment across different systems. The Docker container:
- Provides a pre-configured ROS2 Humble environment with all necessary dependencies
- Ensures compatibility across Linux and macOS development environments
- Eliminates "it works on my machine" issues by standardizing the build environment
- Makes it easy for new developers to get started without manual setup
- Isolates the ROS2 environment from your system's packages

Before proceeding, make sure you have Docker installed and configured. For detailed setup instructions, including prerequisites and troubleshooting, see [DOCKER_SETUP.md](DOCKER_SETUP.md).

## Quick Start

1. Clone the repository:
```bash
git clone git@github.com:speedybits/b4m_turtlebot3.git
cd b4m_turtlebot3
```

2. Set up your B4M API token as described in the Prerequisites section.

3. Run the development launch script:
```bash
./b4m_dev_launch.sh
```

The script will:
- Start the Docker container if it's not running (with all dependencies pre-installed)
- Build the ROS2 packages
- Provide instructions for launching the nodes

4. Follow the launch instructions provided by the script to start the bridge and voice nodes in separate terminals.

## Development Guide

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

2. After making changes, run the development launch script again to rebuild:
```bash
./b4m_dev_launch.sh
```

### Testing Voice Control
Once both nodes are running, you can:

1. Test voice commands:
   - Say "move forward" to initiate forward movement
   - Say "stop" to halt the robot
   - Say "turn left" or "turn right" for rotation
   - Say "status" to get the current robot state

2. Use text commands instead of voice:
```bash
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'move forward'"
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'stop'"
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'turn left'"
ros2 topic pub --once /text_commands std_msgs/msg/String "data: 'status'"
```

### Monitoring the System
Check node and topic status:
```bash
# List running nodes
ros2 node list

# List active topics
ros2 topic list

# Monitor voice commands
ros2 topic echo /voice_commands

# Monitor bridge status
ros2 topic echo /bridge_status
```

## Notes
- The Docker container mounts the workspace directory, so you can edit code on your host machine
- All changes are immediately reflected in the container
- The container uses network_mode: host for easy ROS2 communication with the host

For detailed Docker setup and configuration, see [DOCKER_SETUP.md](DOCKER_SETUP.md).
