export CONTAINER="jetson-ros2-foxy-desktop"
export DOCKERFILE="Dockerfile.ros.foxy"

ROS_DISTRO="foxy" # foxy, eloquent, dashing, melodic, kinetic
ROS_PACKAGE="desktop"


# sudo locale-gen en_US en_US.UTF-8 
# sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export PYTHONIOENCODING=utf-8

sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1


export CURRENT_DIR=`pwd`
echo " - run in $CURRENT_DIR"

# bash docker/scripts/install-opencv.sh $CURRENT_DIR


# source ./docker/scripts/docker_base.sh
# source ./docker/scripts/ros_distro.sh

if [[ ! " ${SUPPORTED_ROS_DISTROS[@]} " =~ " ${ROS_DISTRO} " ]]; then
    echo "${LRED}[Error]${DEFAULT} -- '$ROS_DISTRO' isn't one of the supported ROS packages:"
    echo "  ${SUPPORTED_ROS_DISTROS[@]}"
    exit 1
fi

if [[ ! " ${SUPPORTED_ROS_PACKAGES[@]} " =~ " ${ROS_PACKAGE} " ]]; then
    echo "${LRED}[Error]${DEFAULT} -- '$ROS_PACKAGE' isn't one of the supported ROS packages:"
    echo "  ${SUPPORTED_ROS_PACKAGES[@]}"
    exit 1
fi

# /etc/hosts 写入 185.199.110.133 raw.githubusercontent.com


export ROS_PKG=$ROS_PACKAGE
export ROS_DISTRO=$ROS_DISTRO
export ROS_ROOT=/opt/ros/${ROS_DISTRO}
export ROS_PYTHON_VERSION=3
# bash docker/scripts/ros2_build.sh
