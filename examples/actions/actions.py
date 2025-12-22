"""Launch a server and a client."""
import os

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch a server and a client."""
    # Robustly find the executable relative to this launch file
    dir_path = os.path.dirname(os.path.abspath(__file__))
    server_exec = os.path.join(dir_path, 'server')
    client_exec = os.path.join(dir_path, 'client')

    return launch.LaunchDescription([
        launch_ros.actions.Node(executable=server_exec,
                                output='screen',
                                name='server'),
        launch_ros.actions.Node(executable=client_exec,
                                output='screen',
                                name='client'),
    ])
