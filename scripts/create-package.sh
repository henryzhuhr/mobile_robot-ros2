# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"

mkdir -p $WORKDIR/modules/sensors
cd $WORKDIR/modules/sensors


ros2 pkg create --build-type ament_cmake sensor_joy \
    --dependencies rclcpp std_msgs


ros2 pkg create --build-type ament_python sensor_joy_py \
    --dependencies rclpy std_msgs

