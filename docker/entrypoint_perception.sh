#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f "/workspace/install_perception/setup.bash" ]; then
    source /workspace/install_perception/setup.bash
fi

# Launch Zenoh Bridge in the background if not already running
if ! pgrep -x "zenoh-bridge-ros2dds" > /dev/null; then
    zenoh-bridge-ros2dds -c /workspace/docker/zenoh/perception_rules.json5 &
fi

exec "$@"