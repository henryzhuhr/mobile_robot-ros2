from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    py_video = Node(package="py_video", executable="video_reader", name="t1")
    return LaunchDescription([py_video, ])