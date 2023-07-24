export WORKDIR=$PWD
echo "work dir ${WORKDIR}"
cd $WORKDIR


# -- 创建包 --
cd $WORKDIR/src
# ros2 pkg create --build-type ament_python vision_lanedet_py \
#     --dependencies rclpy sensor_msgs cv_bridge
# exit

# -- 构建运行 --
cd $WORKDIR
rm -rf build install log

# eval "$(conda shell.bash hook)"
# conda activate ros2-development

colcon build
# colcon build --packages-select vision_lanedet_py
# colcon build --packages-select cpp_video_streamer
source install/setup.bash

which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path


# ros2 run vision_lanedet_py lanedet
# ros2 launch vision_lanedet_py lanedet.launch.py
# ros2 launch cpp_video_streamer video_reader.launch.py
ros2 launch py_launch run_all_nodes.launch.py

