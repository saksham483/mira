FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=20
ENV ROS_LOCALHOST_ONLY=1

ENV PATH="/usr/local/bin:/root/.local/bin:${PATH}"

WORKDIR /workspace

# 1. System, Build, Dev Tools & ROS 2 Jazzy Packages
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
    gdb \
    vim \
    nano \
    htop \
    tree \
    net-tools \
    iputils-ping \
    usbutils \
    libtinyxml2-dev \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
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

# 2. Install Zenoh Bridge (GNU standalone binary for aarch64)
ARG ZENOH_VERSION=1.1.1
RUN ARCH=$(uname -m) && \
    curl -fsSL -o /tmp/zenoh.zip \
    "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-${ARCH}-unknown-linux-gnu-standalone.zip" && \
    unzip -q /tmp/zenoh.zip -d /usr/local/bin && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm -f /tmp/zenoh.zip

RUN pip3 install --no-cache-dir --break-system-packages \
    pymavlink \
    pyserial \
    pyyaml

COPY docker/entrypoint_controls.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
