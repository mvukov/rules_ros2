from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2/test/launch_deps/hello'],
            cached_output=True,
        ),
    ])
