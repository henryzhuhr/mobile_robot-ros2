from launch import LaunchDescription
from launch_ros.actions import Node

import platform


def generate_launch_description():
    node_list = []
    video_reader = Node(package="py_video", executable="video_reader", name="t1")
    video_viewer = Node(package="py_video", executable="video_viewer", name="t1")

    # node_list.append(video_reader)
    # node_list.append(video_viewer)

    # 查看是否是 Jetson Nano 平台
    device_platform = platform.machine()
    # if device_platform=="aarch64":

    # else:


    print("add node", node_list)

    return LaunchDescription(node_list)
