# -- 创建包 --
WORKDIR=$PWD
echo "work dir ${WORKDIR}"

mkdir -p $WORKDIR/modules/common
cd $WORKDIR/modules/common


ros2 pkg create pkg_node \
    --build-type ament_cmake \
    --dependencies rclcpp


ros2 pkg create pkg_node_py \
    --build-type ament_python \
    --dependencies rclpy

