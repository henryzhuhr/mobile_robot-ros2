

# ros2 pkg create video --build-type ament_python --dependencies rclpy

colcon build --packages-select py_launch
colcon build --packages-select py_video



colcon build
. install/setup.bash
ros2 run py_video video_reader

colcon build
. install/setup.bash
ros2 run py_video video_viewer

colcon build
. install/setup.bash
ros2 launch py_launch py_all_nodes.launch.py

colcon build &&. install/setup.bash && ros2 launch py_launch py_all_nodes.launch.py