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

# Optional: Install audio dependencies (only used on Linux with real microphone)
ARG INSTALL_AUDIO=false
RUN if [ "$INSTALL_AUDIO" = "true" ] ; then \
        apt-get update && apt-get install -y \
        python3-pyaudio \
        portaudio19-dev \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

# Set up workspace
WORKDIR /workspace
COPY . .

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    pytest \
    flake8 \
    bike4py

# Optional: Install audio Python packages (only used on Linux with real microphone)
RUN if [ "$INSTALL_AUDIO" = "true" ] ; then \
        pip3 install --no-cache-dir \
        SpeechRecognition>=3.8.1 ; \
    fi

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
