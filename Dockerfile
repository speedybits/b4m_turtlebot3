# Use ROS2 Humble as base
FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace
COPY . .

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    pytest \
    flake8 \
    bike4py

# Build the workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash && colcon build --packages-select b4m_bridge b4m_voice'

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
\n\
exec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
