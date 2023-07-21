
export WORKDIR=$PWD/ros-example
echo "work dir ${WORKDIR}"
cd $WORKDIR


# -- 创建包 --
cd $WORKDIR/src
# ros2 pkg create --build-type ament_cmake cpp_parameters \
#     --dependencies rclcpp
# exit

# -- 构建运行 --
cd $WORKDIR
rm -rf build install log

colcon build
# colcon build --packages-select cpp_parameters
source install/setup.bash

# ros2 run cpp_parameters parameter_node
# ros2 launch cpp_parameters cpp_parameters_launch.py
# ros2 launch py_launch run_all_nodes.launch.py