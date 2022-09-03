"""Launch a server and a client."""

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch a talker and a listener."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(executable='examples/actions/server',
                                output='screen',
                                name='server'),
        launch_ros.actions.Node(executable='examples/actions/client',
                                output='screen',
                                name='client'),
    ])
