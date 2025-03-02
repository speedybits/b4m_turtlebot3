#!/bin/bash

# Detect the operating system
OS=$(uname -s)

# Default values
export DOCKER_NETWORK_MODE=bridge
export DOCKER_PRIVILEGED=false
export LIBGL_ALWAYS_INDIRECT=1

if [ "$OS" = "Linux" ]; then
    # Linux-specific settings
    export DOCKER_NETWORK_MODE=host
    export XAUTH=$HOME/.Xauthority
    
    # Check if we're running on WSL
    if grep -qi microsoft /proc/version; then
        echo "WSL detected, using bridge network"
        export DOCKER_NETWORK_MODE=bridge
        export DISPLAY=host.docker.internal:0.0
    fi

    # Set up X11 authentication
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

elif [ "$OS" = "Darwin" ]; then
    # macOS-specific settings
    export DOCKER_NETWORK_MODE=bridge
    export DISPLAY=host.docker.internal:0
    export XAUTH=$HOME/.Xauthority
    
    # Ensure XQuartz is running
    if ! pgrep -x "Xquartz" > /dev/null; then
        echo "XQuartz is not running. Please start XQuartz first:"
        echo "open -a XQuartz"
        exit 1
    fi
    
    # Set up X11 authentication
    touch $XAUTH
    xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# Print the configuration
echo "Docker environment configured for $OS:"
echo "DOCKER_NETWORK_MODE=$DOCKER_NETWORK_MODE"
echo "DOCKER_PRIVILEGED=$DOCKER_PRIVILEGED"
echo "DISPLAY=$DISPLAY"
echo "XAUTH=$XAUTH"
echo "LIBGL_ALWAYS_INDIRECT=$LIBGL_ALWAYS_INDIRECT"

# Export variables for docker-compose
export DISPLAY
export XAUTH
