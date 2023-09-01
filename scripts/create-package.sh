# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"

mkdir -p $WORKDIR/core/
cd $WORKDIR/core/


# ros2 pkg create motion_controller \
#     --build-type ament_cmake \
#     --dependencies rclcpp


ros2 pkg create roslaunch \
    --build-type ament_python \
    --dependencies rclpy

