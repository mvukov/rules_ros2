from ros2.houston import houston


def create_deployment():
    return houston.Deployment([
        houston.ParametersFile('ros2/houston/global.yaml'),
        houston.EnvironmentVariable('foo', 'bar'),
        houston.EnvironmentVariable('ROS_LOG_DIR', '/tmp'),
        houston.Node('talker',
                     'ros2/houston/talker',
                     parameters_file='ros2/houston/config.yaml'),
        houston.Node('listener',
                     'ros2/houston/listener',
                     parameters_file='ros2/houston/config2.yaml'),
    ])
