#!/bin/bash
set -e

unset VIRTUAL_ENV
unset PYTHONHOME

source /opt/ros/jazzy/setup.bash
if [ -f "/workspace/install_controls/setup.bash" ]; then
    source /workspace/install_controls/setup.bash
fi

# Launch Zenoh bridge on Domain 20
if command -v zenoh-bridge-ros2dds > /dev/null && [ -f "/workspace/docker/zenoh/controls_rules.json5" ]; then
    if ! pgrep -f "zenoh-bridge-ros2dds" > /dev/null; then
        zenoh-bridge-ros2dds -c /workspace/docker/zenoh/controls_rules.json5 &
    fi
fi

exec "$@"
