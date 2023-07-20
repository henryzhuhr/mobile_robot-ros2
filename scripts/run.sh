
cd ros
rm -rf build install log
colcon build
. install/setup.bash
ros2 launch py_launch py_all_nodes.launch.py






cd src
ros2 pkg create --build-type ament_cmake cpp_video_streamer
cd ..

colcon build --packages-select cpp_video_streamer

✨ feat: ros video_streamer (cpp, opencv)