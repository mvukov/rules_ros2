import launch.actions
import launch_ros.actions

import third_party.foxglove_bridge.node_path

# Extracted from ros2_foxglove_bridge/launch/foxglove_bridge_launch.xml
PARAMS_TO_DEFAULT_VALUES = {
    'port':
        8765,
    'address':
        '0.0.0.0',
    'tls':
        False,
    'certfile':
        '',
    'keyfile':
        '',
    'topic_whitelist': ['.*'],
    'param_whitelist': ['.*'],
    'service_whitelist': ['.*'],
    'client_topic_whitelist': ['.*'],
    'max_qos_depth':
        10,
    'num_threads':
        0,
    'send_buffer_limit':
        10000000,
    'use_sim_time':
        False,
    'capabilities': [
        'clientPublish',
        'connectionGraph',
        'parameters',
        'parametersSubscribe',
        # For services we need auto-generated .msg files. TBD.
        # 'services',
    ],
}


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='ROS_DISTRO',
                                              value='humble'),
        launch_ros.actions.Node(
            # Provide the rootpath for the node.
            executable=third_party.foxglove_bridge.node_path.NODE_PATH,
            output='screen',
            parameters=[
                PARAMS_TO_DEFAULT_VALUES,
            ],
            prefix=[
                'gdb -ex "break abort" -ex "thread apply all bt" -ex "run" -ex "bt" --args'  # noqa
            ],  # noqa
        ),
    ])
