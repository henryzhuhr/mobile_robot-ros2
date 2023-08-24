# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"

mkdir -p $WORKDIR/sources/common
cd $WORKDIR/sources/common


ros2 pkg create --build-type ament_cmake system_state \
    --dependencies rclcpp std_msgs


ros2 pkg create --build-type ament_python system_state_py \
    --dependencies rclpy std_msgs

