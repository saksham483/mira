FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=10
ENV ROS_LOCALHOST_ONLY=1
WORKDIR /workspace

# 1. System, Boost.Python, and ROS 2 Humble build dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    unzip \
    curl \
    pkg-config \
    python3-dev \
    libboost-dev \
    libboost-python-dev \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    gstreamer1.0-plugins-base-apps \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    libgstreamer-plugins-good1.0-dev \
    libunwind-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Zenoh Bridge (GNU standalone binary)
ARG ZENOH_VERSION=1.1.1
RUN ARCH=$(uname -m) && \
    curl -fsSL -o /tmp/zenoh.zip \
    "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-${ARCH}-unknown-linux-gnu-standalone.zip" && \
    unzip -q /tmp/zenoh.zip -d /usr/local/bin && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm -f /tmp/zenoh.zip

# 3. Python dependencies:
# - Pre-install PyTorch CPU to avoid pulling 4GB of CUDA wheels
# - Pin setuptools to 70.3.0 to fix the canonicalize_version error
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir "setuptools==70.3.0" "packaging>=24.0" && \
    pip3 install --no-cache-dir \
    ultralytics \
    scipy \
    opencv-python \
    onnxruntime

# 4. Entrypoint
COPY docker/entrypoint_perception.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]