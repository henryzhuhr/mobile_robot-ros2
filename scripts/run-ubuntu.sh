source scripts/constant.sh

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

source ~/.bashrc
source /opt/ros/foxy/setup.bash

eval "$(conda shell.bash hook)"
conda activate ros2

# pip install empy numpy==1.20


BUILD_LIST=(
    # "interfaces" # 统一接口
    "cpp_video_streamer" # 视频流
    # "vision_lanedet_interfaces"
    # "car_controller_py"
    "py_launch"
)
for item in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${item}${DEFAULT}"
    colcon build --packages-select ${item}
    source install/setup.bash
done

which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path



ros2 launch py_launch cpp_video_streamer-video_reader.launch.py
# ros2 launch py_launch run_all_nodes.launch.py


