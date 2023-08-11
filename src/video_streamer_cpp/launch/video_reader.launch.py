from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="video_streamer_cpp",   # package
                executable="video_reader",      # node
                name="t1",
                parameters=[{
                    "source": "camera", # camera or file/url
                }],
            )
        ]
    )
