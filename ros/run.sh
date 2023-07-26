
# https://pypi.tuna.tsinghua.edu.cn/simple/setuptools/
pip3 install resource/setuptools-58.2.0-py3-none-any.whl
pip3 install resource/pyserial-3.5-py2.py3-none-any.whl
# numpy==1.20.2

rm -rf install build log

source ~/.bashrc
source /opt/ros/humble/install/setup.bash

# colcon build
colcon build --packages-select vision_lanedet_interfaces
colcon build --packages-select car_controller_py

source install/setup.bash
# ros2 launch py_launch run_all_nodes.launch.py

ros2 run car_controller_py auto_control_lanedet