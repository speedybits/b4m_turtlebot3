# B4M TurtleBot3 Project

This repository contains the ROS2 packages for the B4M TurtleBot3 project.

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

3. Build and run the project:
```bash
docker compose exec ros2_dev bash
colcon build
source install/setup.bash
```

For detailed setup instructions, please refer to [DOCKER_SETUP.md](DOCKER_SETUP.md).

# To run the simulation
./b4m_launch.sh
