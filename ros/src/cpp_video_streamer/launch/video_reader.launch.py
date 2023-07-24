from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cpp_video_streamer",   # package
                executable="video_reader",      # node
                name="t1",
                parameters=[{
                    "source": "camera", # camera or file/url
                }],
            )
        ]
    )
