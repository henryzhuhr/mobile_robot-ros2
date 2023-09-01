from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(     
        package="joy",
        executable="joy_node",
        name="joy",
    )
    sensor_joy = Node(
        package="sensor_joy_py",
        executable="sensor_joy",
        name="sensor_joy",
    )
    launch_description = LaunchDescription(
        [joy_node,sensor_joy])
    return launch_description