
cd ros
rm -rf build install log
colcon build
. install/setup.bash
ros2 launch py_launch py_all_nodes.launch.py




colcon build --packages-select cpp_ju_video