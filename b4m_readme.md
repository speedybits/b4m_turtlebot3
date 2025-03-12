# B4M TurtleBot3 Project

This repository contains the ROS2 packages for the B4M TurtleBot3 project, focusing on:
- `b4m_bridge`: ROS2 bridge node for processing sensor data and controlling the robot
- `b4m_voice`: Voice control package for natural language interaction

## Prerequisites

1. Docker installed and configured (see [DOCKER_SETUP.md](DOCKER_SETUP.md))
   - **Mac Users**: Make sure to start Docker Desktop from Launchpad before running any commands
2. B4M API token (required for robot communication)
3. For Ubuntu Linux only: Working microphone (optional, for voice control with real audio input)

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

The voice control system's dependencies vary based on your operating system and usage mode, all of which are automatically installed in the Docker image:

#### For All Users (Mac and Linux)
No additional Python packages are required for simulation mode, which uses ROS2 topics for command input.

#### For Ubuntu Linux Users (Optional)
Only needed when using real microphone input:
- Python SpeechRecognition library (for processing real voice input)
- PyAudio for microphone input
- PortAudio system library

Note: Mac users run in simulation mode only and don't need any of these audio-related dependencies.

### Voice Control Setup

The voice control system has different requirements and capabilities depending on your operating system:

#### macOS Users
On macOS, voice control operates in simulation mode only and does not use a microphone:
- No microphone setup is required
- Commands are simulated through software
- Perfect for testing and development

```bash
# Launch with simulation mode (macOS default)
ros2 launch b4m_voice b4m_voice.launch.py
```

#### Ubuntu Linux Users
On Ubuntu, you can use both simulation mode and real microphone input for voice commands:

1. Setup audio permissions for microphone access:
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

1. Follow the setup instructions in [DOCKER_SETUP.md](DOCKER_SETUP.md) to install Docker and configure your environment.
   - **Mac Users**: Ensure Docker Desktop is running by launching it from your Applications or Launchpad
   - **Linux Users**: Ensure the Docker daemon is running with `sudo systemctl status docker`

2. Set up your B4M API token as described in the "Setting up B4M API Token" section above.

3. Use the development launch script to set up the environment:
```bash
./b4m_dev_launch.sh
```
This script will:
- Start the Docker container if it's not running
- Build the ROS2 packages
- Show instructions for launching nodes and sending commands

4. Launch the nodes in separate terminals:
```bash
# Terminal 1: Launch the bridge node
./b4m_dev_launch.sh bridge

# Terminal 2: Launch the voice control node
./b4m_dev_launch.sh voice
```

5. Send voice commands from any terminal:
```bash
# Move the robot forward
./b4m_dev_launch.sh send "move forward"

# Stop the robot
./b4m_dev_launch.sh send "stop"

# See all available commands
./b4m_dev_launch.sh help
```

### Available Voice Commands
- "move forward" - Start moving forward
- "stop" - Stop movement
- "turn left" - Rotate left
- "turn right" - Rotate right
- "status" - Get robot status

### Manual Launch (Alternative)

If you need to launch components manually instead of using the script:

1. Start a new terminal for the bridge node:
```bash
docker compose exec ros2_dev bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 run b4m_bridge b4m_bridge
```

2. Start another terminal for the voice control node:
```bash
docker compose exec ros2_dev bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 run b4m_voice voice_control
```

Note: We recommend using `b4m_dev_launch.sh` as it simplifies the launch process into single commands.

### Testing Voice Control
Once both nodes are running, you can:

1. Test voice commands (Linux only):
   - Say "move forward" to initiate forward movement
   - Say "stop" to halt the robot
   - Say "turn left" or "turn right" for rotation

2. Test simulated commands (Mac and Linux):
   - Use the ROS2 command line to publish commands:
     ```bash
     ros2 topic pub /speaker/speech_input std_msgs/msg/String "data: 'move forward'"
     ros2 topic pub /speaker/speech_input std_msgs/msg/String "data: 'stop'"
     ```

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
