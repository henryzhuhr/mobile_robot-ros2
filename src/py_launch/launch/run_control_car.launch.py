from launch import LaunchDescription
from launch_ros.actions import Node

import platform

platform_architecture = platform.architecture()[0]


def generate_launch_description():

    car_control_node= Node(# 「节点」 车辆控制
        package="car_controller_py",
        executable="auto_control",
        name="car_controller_py",
    )
    joy_node = Node(           # 「节点」 手柄控制
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    return LaunchDescription([
        car_control_node,
        joy_node,
    ])
