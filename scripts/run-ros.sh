export WORKDIR=$PWD
echo "work dir ${WORKDIR}"
cd $WORKDIR


# -- 创建包 --
cd $WORKDIR/src
# ros2 pkg create --build-type ament_cmake car_controller \
#     --dependencies rclpy sensor_msgs cv_bridge
# exit

# -- 构建运行 --
cd $WORKDIR
# rm -rf build install log

eval "$(conda shell.bash hook)"
conda activate ros2

# pip install empy numpy==1.20
# colcon build
# colcon build --packages-select interfaces
# colcon build --packages-select py_launch
# colcon build --packages-select cpp_video_streamer
# colcon build --packages-select vision_lanedet_py
colcon build --packages-select car_controller_py


source install/setup.bash

which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path



ros2 launch py_launch run_control_car.launch.py
# ros2 launch py_launch run_lanedet.launch.py
# ros2 launch py_launch run_all_nodes.launch.py


