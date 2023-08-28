# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"

mkdir -p $WORKDIR/modules/sensors
cd $WORKDIR/modules/sensors


ros2 pkg create system_state \
    --build-type ament_cmake \
    --dependencies rclcpp


ros2 pkg create system_state_py \
    --build-type ament_python \
    --dependencies rclpy

