
# https://pypi.tuna.tsinghua.edu.cn/simple/setuptools/
pip3 install resource/setuptools-58.2.0-py3-none-any.whl
# numpy==1.20.2

rm -rf install build log
colcon build
source ~/.bashrc
source /opt/ros/humble/install/setup.bash
source install/setup.bash
ros2 launch py_launch py_all_nodes.launch.py