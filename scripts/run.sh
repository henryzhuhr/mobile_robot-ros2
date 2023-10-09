source scripts/constant.sh

echo "${LGREEN}Find [/dev/ttyUSB]: \n$(ls /dev/ttyUSB*)${DEFAULT}"
echo "${LGREEN}Find [/dev/ttyTHS]: \n$(ls /dev/ttyTHS*)${DEFAULT}"
echo "${LGREEN}Find [/dev/ttyACM]: \n$(ls /dev/ttyACM*)${DEFAULT}"

# https://pypi.tuna.tsinghua.edu.cn/simple/setuptools/
pip3 install $public_dir/setuptools-58.2.0-py3-none-any.whl
pip3 install $public_dir/pyserial-3.5-py2.py3-none-any.whl
# numpy==1.20.2

rm -rf install build log

source ~/.bashrc
source /opt/ros/foxy/install/setup.bash
source $CONDA_PREFIX/setup.zsh

BUILD_LIST=(
    "interfaces" # 统一接口
    "video_streamer_cpp" # 视频流
    "vision_lanedet_py"     # 视觉 车道线检测
    "controller_py"
    "py_launch"
)
for item in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${item}${DEFAULT}"
    colcon build --packages-select ${item} \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug
    source install/setup.bash
done
source install/setup.bash


which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path

# ros2 launch py_launch cpp_video_streamer-video_reader.launch.py
ros2 launch py_launch car.launch.py
