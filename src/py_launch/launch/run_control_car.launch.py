from launch import LaunchDescription
from launch_ros.actions import Node

import platform

platform_architecture = platform.architecture()[0]


def generate_launch_description():

    cpp_video_reader = Node(
        package="cpp_video_streamer",
        executable="video_reader",
        name="t1",
        parameters=[{
            "source": "camera", # camera or file/url
        }],
    )
    cpp_video_viewer = Node(
        package="cpp_video_streamer",
        executable="video_viewer",
        name="t1",
    )
    car_control_node= Node(                 # 「节点」 车辆控制
        package="car_controller_py",
        executable="auto_control",
        name="car_controller_py",
    )
    joy_node = Node(                        # 「节点」 手柄控制
        package="joy",
        executable="joy_node",
        name="joy_node",
    )
    
    """
    trtexec --fp16 \
    --onnx=public/weights/train/ufld-final-INT32.onnx \
    --saveEngine=public/weights/train/ufld-final-INT32.engine
    """
    weight_file = "public/weights/train/ufld-final-INT32.engine"
    if platform.machine() == "aarch64":
        weight_file = "weights/ufld-final-INT32-jetsonnano.engine"
    vision_lanedet_node = Node(             # 「节点」 视觉 车道线检测
        package="vision_lanedet_py",
        executable="lane_detector",
        name="vision_lanedet_py",
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
