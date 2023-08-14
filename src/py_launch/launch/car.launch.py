from launch import LaunchDescription
from launch_ros.actions import Node

import platform

platform_architecture = platform.architecture()[0]


def generate_launch_description():
    joy_node = Node(                        # 「节点」 手柄控制
        package="joy",
        executable="joy_node",
        name="joy",
    )
    car_control_node = Node(                 # 「节点」 车辆控制
        package="controller_py",
        executable="control",
        name="control",
        parameters=[{
            "joy_config": "configs/joys/Microsoft-X-Box-360-pad.json",
        }],
    )
    cpp_video_reader = Node(
        package="video_streamer_cpp",
        executable="video_reader",
        name="videoR",
        parameters=[{
            "source": "camera2",  # camera or file/url
        }],
    )
    cpp_video_viewer = Node(
        package="video_streamer_cpp",
        executable="video_viewer",
        name="videoV",
    )

    """
    trtexec --fp16 \
    --onnx=public/weights/train/ufld-final-INT32.onnx \
    --saveEngine=public/weights/train/ufld-final-INT32.engine
    """
    weight_file = "public/weights/culane_18-INT32.onnx"
    if platform.machine() == "aarch64":
        # weight_file = "weights/ufld-final-INT32-jetsonnano.engine"
        weight_file = "public/weights/culane_18-INT32.onnx"
    vision_lanedet_node = Node(             # 「节点」 视觉 车道线检测
        package="vision_lanedet_py",
        executable="lane_detector",
        name="lanedet",
        parameters=[{
            "weight_file": weight_file,
            "skip_frame": 5,
            "img_h": 720,
            "img_w": 1280,
        }],
    )

    return LaunchDescription([
        cpp_video_reader,
        cpp_video_viewer,
        car_control_node,
        joy_node,
        vision_lanedet_node,
    ])
