

CONTAINER="jetson-ros2-foxy-desktop"
DOCKERFILE="Dockerfile.ros.foxy"

ROS_DISTRO="foxy" # foxy, eloquent, dashing, melodic, kinetic
ROS_PACKAGE="desktop"
WITH_PYTORCH="off"


source ./docker/scripts/color.sh
source ./docker/scripts/docker_base.sh
source ./docker/scripts/opencv_version.sh
source ./docker/scripts/ros_distro.sh


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


sudo docker build \
    -f docker/containers/$DOCKERFILE \
    -t $CONTAINER \
    --network=host \
    --build-arg ROS_PKG=$ROS_PACKAGE \
    --build-arg ROS_VERSION=$ROS_DISTRO \
    --build-arg OPENCV_URL=$OPENCV_URL \
    --build-arg OPENCV_DEB=$OPENCV_DEB \
    .