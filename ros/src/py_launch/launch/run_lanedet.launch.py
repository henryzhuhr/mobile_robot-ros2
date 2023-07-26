from launch import LaunchDescription
from launch_ros.actions import Node

import platform


def generate_launch_description():
    node_list = []
    video_reader = Node(package="py_video_streamer", executable="video_reader", name="t1")
    video_viewer = Node(package="py_video_streamer", executable="video_viewer", name="t1")

    cpp_video_reader = Node(
        package="cpp_video_streamer",
        executable="video_reader",
        name="t1",
        parameters=[{
            "source": "weights/test.mp4", # camera or file/url
        }],
    )
    cpp_video_viewer = Node(
        package="cpp_video_streamer",
        executable="video_viewer",
        name="t1",
    )

    vision_lanedet_node = Node(# 「节点」 视觉 车道线检测
        package="vision_lanedet_py",
        executable="lane_detector",
        name="vision_lanedet_py",
        parameters=[{
            "weight_file": "weights/ufld-final-x64.engine",
            "video": "weights/test.mp4",
            "skip_frame": 5,
            "img_h": 720,
            "img_w": 1280,
        }],
    )

    car_control_node= Node(# 「节点」 车辆控制
        package="car_controller_py",
        executable="auto_control_lanedet",
        name="car_controller_py",
    )

    return LaunchDescription([
        cpp_video_reader,
                               # cpp_video_viewer,
        vision_lanedet_node,
        car_control_node,
    ])
