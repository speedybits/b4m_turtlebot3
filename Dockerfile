# Use ROS2 Humble as base
FROM ros:humble-ros-base

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace
COPY . .

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    pytest \
    flake8 \
    pytest-cov \
    bike4py

# Build the workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash && colcon build'

# Create entrypoint script
RUN echo '#!/bin/bash\n\
\n\
# Source ROS environment\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
\n\
# Execute the command passed to docker run\n\
exec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
