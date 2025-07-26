from ros2.houston import houston


def create_deployment():
    return houston.Deployment([
        houston.ParametersFile('ros2/houston/global.yaml'),
        houston.EnvironmentVariable('foo', 'bar'),
        houston.Node('apple',
                     'cmd1',
                     parameters_file='ros2/houston/config.yaml'),
        houston.Node('hohoho',
                     'cmd2',
                     parameters_file='ros2/houston/config2.yaml'),
    ])
