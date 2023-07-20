from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    video_reader = Node(package="py_video", executable="video_reader", name="t1")
    video_viewer = Node(package="py_video", executable="video_viewer", name="t1")

    return LaunchDescription([video_reader,video_viewer,])