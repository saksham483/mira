FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=20
WORKDIR /workspace

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    git \
    lld \
    ninja-build \
    pkg-config \
    ccache \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-cyclonedds \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-vision-msgs \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Zenoh Bridge
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee /etc/apt/sources.list.d/zenoh.list && \
    apt-get update && apt-get install -y zenoh-bridge-ros2dds && \
    rm -rf /var/lib/apt/lists/*

# Install uv & Python dependencies
COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/
RUN uv pip install --system pymavlink pyserial pyyaml

# Entrypoint
COPY docker/entrypoint_controls.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]