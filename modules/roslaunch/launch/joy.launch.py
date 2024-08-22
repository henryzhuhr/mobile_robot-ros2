from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(# ROS 内置节点
        package="joy",
        executable="joy_node",
        name="joy",
    )
    sensor_joy = Node( # 「节点」 手柄控制
        package="sensor_joy_py",
        executable="sensor_joy",
        name="sensor_joy",
    )
    motion_controller=Node( # 「节点」 运动控制器
        package="motion_controller_py",
        executable="motion_controller",
        name="motion_controller",
    )
    return LaunchDescription([
       joy_node,
       sensor_joy,
       motion_controller
    ])
