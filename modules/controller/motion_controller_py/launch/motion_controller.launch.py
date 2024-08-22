from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    # FROM core/controller/motion_controller_py/setup.py
    motion_controller_node = Node(
        package="motion_controller_py",
        executable="motion_controller",
        name="motion_controller",
    )
    test_motion_controller_node = Node(
        package="motion_controller_py",
        executable="test_motion_controller",
        name="test_motion_controller",
    )
    return LaunchDescription([
        motion_controller_node,test_motion_controller_node
    ])
