source scripts/constant.sh

export WORKDIR=$PWD
echo "work dir ${WORKDIR}"
cd $WORKDIR


# -- 创建包 --
cd $WORKDIR/src
# ros2 pkg create --build-type ament_python control_panel_flask_vue \
#     --dependencies rclpy std_msgs sensor_msgs cv_bridge
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
    "video_streamer_cpp" # 视频流
    # "vision_lanedet_py"     # 视觉 车道线检测
    # "controller_py"
    # "py_launch"
)
for item in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${item}${DEFAULT}"
    colcon build --packages-select ${item} \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug
    source install/setup.bash
done

which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path

# ros2 launch py_launch cpp_video_streamer-video_reader.launch.py
ros2 launch py_launch car.launch.py


