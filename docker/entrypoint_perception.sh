#!/bin/bash
set -e

export ROS_LOCALHOST_ONLY=1

source /opt/ros/humble/setup.bash
if [ -f "/workspace/install_perception/setup.bash" ]; then
    source /workspace/install_perception/setup.bash
fi

# Launch Zenoh bridge if present and rules file exists
if command -v zenoh-bridge-ros2dds > /dev/null && [ -f "/workspace/docker/zenoh/perception_rules.json5" ]; then
    if ! pgrep -f "zenoh-bridge-ros2dds" > /dev/null; then
        zenoh-bridge-ros2dds --ros-localhost-only -c /workspace/docker/zenoh/perception_rules.json5 > /tmp/zenoh-bridge.log 2>&1 &
    fi
fi

exec "$@"