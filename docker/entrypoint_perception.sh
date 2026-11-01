#!/bin/bash
set -e

# Clear any virtualenv flags inherited from the host
unset VIRTUAL_ENV
unset PYTHONHOME

source /opt/ros/humble/setup.bash
if [ -f "/workspace/install_perception/setup.bash" ]; then
    source /workspace/install_perception/setup.bash
fi

# Launch Zenoh bridge if config exists and process not running
if command -v zenoh-bridge-ros2dds > /dev/null && [ -f "/workspace/docker/zenoh/perception_rules.json5" ]; then
    if ! pgrep -f "zenoh-bridge-ros2dds" > /dev/null; then
        zenoh-bridge-ros2dds -c /workspace/docker/zenoh/perception_rules.json5 &
    fi
fi

exec "$@"
