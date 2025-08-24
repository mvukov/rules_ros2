from ros2.houston import entity


def create_deployment():
    return entity.Deployment([
        entity.ParametersFile(path='ros2/houston/global.yaml'),
        entity.EnvironmentVariable(name='foo', value='bar'),
        entity.EnvironmentVariable(name='ROS_LOG_DIR', value='/tmp'),
        entity.RosNode(name='talker',
                       executable='ros2/houston/talker',
                       parameters_file='ros2/houston/config.yaml'),
        entity.RosNode(name='listener',
                       executable='ros2/houston/listener',
                       parameters_file='ros2/houston/config2.yaml'),
    ])
