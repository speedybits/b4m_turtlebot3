#!/bin/bash

# Remove any existing .docker.xauth file
rm -f /tmp/.docker.xauth

# Create the xauth file
touch /tmp/.docker.xauth

# Generate xauth token
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -

# Set permissions
chmod 777 /tmp/.docker.xauth
