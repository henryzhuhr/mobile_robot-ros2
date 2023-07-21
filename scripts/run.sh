




# cd ros
# rm -rf build install log
# colcon build --packages-select cpp_video_streamer
# colcon build --packages-select py_launch
# source install/setup.bash
# ros2 launch py_launch run_all_nodes.launch.py






# cd src
# ros2 pkg create --build-type ament_cmake cpp_video_streamer
# cd ..

# colcon build --packages-select cpp_video_streamer
# . install/setup.bash
# ros2 run cpp_video_streamer video_reader
