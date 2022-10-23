"""Launch a server and a client."""
import launch
import launch_ros.actions


def generate_launch_description():
    """Launch a server and a client."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(executable='actions/server',
                                output='screen',
                                name='server'),
        launch_ros.actions.Node(executable='actions/client',
                                output='screen',
                                name='client'),
    ])
