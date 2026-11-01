# 1. Base image with ZED SDK 4.x + CUDA 12.x + PyZED for JetPack 6 (L4T R36.x)
FROM stereolabs/zed:4.2-py-devel-l4t-r36.4

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=10
ENV ROS_LOCALHOST_ONLY=1

# Permanent library paths for CUDA 12.6, CUPTI, and cuDSS
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/libcudss/12:/usr/local/cuda-12.6/lib64:/usr/local/cuda-12.6/extras/CUPTI/lib64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}

WORKDIR /workspace

# 2. Add NVIDIA CUDA 12.6 official repository
RUN apt-get update && apt-get install --no-install-recommends -y curl wget gnupg lsb-release && \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm -f /tmp/cuda-keyring.deb

# 3. Add ROS 2 Humble APT repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list

# 4. Install System, ROS 2 Humble, GStreamer, and CUDA/TensorRT tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    unzip \
    pkg-config \
    python3-dev \
    libboost-dev \
    libboost-python-dev \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ros-base \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    gstreamer1.0-plugins-base-apps \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    libgstreamer-plugins-good1.0-dev \
    libunwind-dev \
    libopenblas-dev \
    cuda-cupti-12-6 \
    libcudss0-cuda-12 \
    libcudss0-dev-cuda-12 \
    python3-libnvinfer \
    python3-libnvinfer-dev \
    && rm -rf /var/lib/apt/lists/*

# 5. Fix cuDSS symlinks and refresh linker cache
RUN mkdir -p /usr/local/cuda/lib64 && \
    ln -sf /usr/lib/aarch64-linux-gnu/libcudss/12/libcudss.so.0 /usr/local/cuda/lib64/libcudss.so.0 && \
    ln -sf /usr/lib/aarch64-linux-gnu/libcudss/12/libcudss.so.0 /usr/lib/aarch64-linux-gnu/libcudss.so.0 && \
    ldconfig

# 6. Install Zenoh Bridge binary
ARG ZENOH_VERSION=1.1.1
RUN ARCH=$(uname -m) && \
    curl -fsSL -o /tmp/zenoh.zip \
    "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-${ARCH}-unknown-linux-gnu-standalone.zip" && \
    unzip -q /tmp/zenoh.zip -d /usr/local/bin && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm -f /tmp/zenoh.zip

# 7. Install Python AI Stack (PyTorch cu126 + ONNX tools + Ultralytics)
RUN pip3 install --no-cache-dir "setuptools==70.3.0" "packaging>=24.0" && \
    pip3 install --no-cache-dir \
    torch torchvision \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 && \
    pip3 install --no-cache-dir \
    onnxruntime-gpu \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 && \
    pip3 install --no-cache-dir \
    "onnx>=1.12.0,<2.0.0" \
    "onnxslim>=0.1.82" \
    "numpy==1.26.4" \
    ultralytics \
    scipy

# 8. Entrypoint
COPY docker/entrypoint_perception.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
