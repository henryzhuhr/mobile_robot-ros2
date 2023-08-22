# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"

mkdir -p $WORKDIR/sources/data_transmission
cd $WORKDIR/sources/data_transmission

ros2 pkg create --build-type ament_python dt_rtmp_py \
    --dependencies rclpy std_msgs

ros2 pkg create --build-type ament_cmake dt_rtmp \
    --dependencies rclcpp std_msgs
