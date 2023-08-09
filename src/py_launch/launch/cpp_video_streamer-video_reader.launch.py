from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_video_streamer",
            executable="video_reader",
            name="t1",
            parameters=[{
                "source": "camera2",  # camera or file/url
            }],
        )
    ])
