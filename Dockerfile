FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    vim \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-transforms3d \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    nlohmann-json3-dev \
    ros-humble-ros2-control-test-assets \
    ros-humble-joint-state-publisher-gui \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libx11-6 \
    libxcb1 \
    libxrender1 \
    libxext6 \
    libxkbcommon-x11-0 \
    libglfw3 \
    libglfw3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    mujoco \
    numpy \
    scipy \
    jsonschema

RUN rosdep init || true && rosdep update --rosdistro humble || true

WORKDIR /ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
