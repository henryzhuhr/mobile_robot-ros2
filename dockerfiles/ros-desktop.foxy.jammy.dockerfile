# https://github.com/osrf/docker_images/blob/20e3ba685bb353a3c00be9ba01c1b7a6823c9472/ros/foxy/ubuntu/focal/desktop/Dockerfile


# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:foxy-ros-base-focal

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# 设置环境为非交互模式，否则在安装某个库时需要选择配置
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y libxcb-keysyms1-dev libxcb-image0-dev \
    libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev libxcb-xfixes0-dev \
    libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev \
    libfontconfig1-dev libfreetype6-dev libx11-dev libxext-dev libxfixes-dev \
    libxi-dev libxrender-dev libxcb1-dev libx11-xcb-dev libxcb-glx0-dev x11vnc \
    xauth build-essential mesa-common-dev libglu1-mesa-dev libxkbcommon-dev \
    libxcb-xkb-dev libxslt1-dev libxkbcommon-x11-0

# launch ros package
# CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]

ADD . /root/ros2-development
WORKDIR /root/ros2-development