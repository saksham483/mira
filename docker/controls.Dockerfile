FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=20
ENV ROS_LOCALHOST_ONLY=1
WORKDIR /workspace

# 1. System, C++, and ROS 2 Jazzy build dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    git \
    lld \
    ninja-build \
    pkg-config \
    ccache \
    unzip \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    libtinyxml2-dev \
    ros-jazzy-cyclonedds \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-vision-msgs \
    ros-jazzy-joy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# 2. Install matching Zenoh Bridge version (GNU standalone binary)
ARG ZENOH_VERSION=1.1.1
RUN ARCH=$(uname -m) && \
    curl -fsSL -o /tmp/zenoh.zip \
    "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-${ARCH}-unknown-linux-gnu-standalone.zip" && \
    unzip -q /tmp/zenoh.zip -d /usr/local/bin && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm -f /tmp/zenoh.zip

# 3. Python MAVLink & Serial communication packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pymavlink \
    pyserial \
    pyyaml

# 4. Entrypoint
COPY docker/entrypoint_controls.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]