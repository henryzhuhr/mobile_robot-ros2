from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(                        # ros2内置节点
        package="joy",
        executable="joy_node",
        name="joy",
    )
    sensor_joy = Node(
        package="sensor_joy_py",
        executable="sensor_joy",
        name="sensor_joy",
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [joy_node,sensor_joy])
    # 返回让ROS2根据launch描述执行节点
    return launch_description