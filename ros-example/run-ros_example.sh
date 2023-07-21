
export WORKDIR=$PWD/ros-example
echo "work dir ${WORKDIR}"
cd $WORKDIR


# 创建包
cd $WORKDIR/src
# ros2 pkg create --build-type ament_cmake cpp_srvcli \
#     --dependencies rclcpp custom_interfaces

cd $WORKDIR
rm -rf build install log
# colcon build 
# source install/setup.bash
colcon build --packages-select custom_interfaces
colcon build --packages-select cpp_srvcli
source install/setup.bash
# ros2 launch py_launch run_all_nodes.launch.py