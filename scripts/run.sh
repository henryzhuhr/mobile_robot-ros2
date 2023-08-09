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
source /opt/ros/humble/install/setup.bash

BUILD_LIST=(
    "cpp_video_streamer" # 视频流
    # "vision_lanedet_interfaces"
    # "car_controller_py"
    # "py_launch"
)
for item in ${BUILD_LIST[@]}; do
    echo "${LGREEN}Build: ${item}${DEFAULT}"
    colcon build --packages-select ${item}
done
source install/setup.bash

# ros2 launch py_launch run_all_nodes.launch.py

# ros2 run car_controller_py auto_control_lanedet

ros2 launch py_launch run_manul_control_car.launch.py