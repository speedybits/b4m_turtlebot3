# Use ROS2 Humble as base image
FROM ros:humble-ros-base

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-webots-ros2 \
    ros-humble-turtlebot3* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install Webots (with no interactive prompts)
RUN wget -q https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y ./webots_2023b_amd64.deb \
    && rm webots_2023b_amd64.deb

# Set up workspace
WORKDIR /workspace
COPY . .

# Build the workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash && colcon build'

# Setup environment
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

# Set environment variables
ENV TURTLEBOT3_MODEL=waffle
ENV WEBOTS_HOME=/usr/local/webots

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
