from ros2.houston import houston


def create_deployment():
    return houston.Deployment([
        houston.ParametersFile(path='ros2/houston/global.yaml'),
        houston.EnvironmentVariable(name='foo', value='bar'),
        houston.EnvironmentVariable(name='ROS_LOG_DIR', value='/tmp'),
        houston.Node(name='talker',
                     executable='ros2/houston/talker',
                     parameters_file='ros2/houston/config.yaml'),
        houston.Node(name='listener',
                     executable='ros2/houston/listener',
                     parameters_file='ros2/houston/config2.yaml'),
    ])
