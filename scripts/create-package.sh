# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"
mkdir -p $WORKDIR/sources/manager
cd $WORKDIR/sources/manager
ros2 pkg create --build-type ament_python system_manager_py \
    --dependencies rclpy std_msgs
ros2 pkg create --build-type ament_cmake system_manager \
    --dependencies rclpy std_msgs
