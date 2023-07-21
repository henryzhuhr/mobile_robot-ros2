from launch import LaunchDescription
from launch_ros.actions import Node

import platform


def generate_launch_description():
    node_list = []
    video_reader = Node(package="py_video_streamer", executable="video_reader", name="t1")
    video_viewer = Node(package="py_video_streamer", executable="video_viewer", name="t1")
    cpp_video_reader = Node(package="cpp_video_streamer", executable="video_reader", name="t1")
    cpp_video_viewer = Node(package="cpp_video_streamer", executable="video_viewer", name="t1")

    node_list.append(cpp_video_reader)
    node_list.append(cpp_video_viewer)


    return LaunchDescription(node_list)
