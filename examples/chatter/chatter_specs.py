from ros2.houston import entity


def create_deployment() -> entity.Deployment:
    return entity.Deployment([
        entity.ParametersFile(path='chatter/params.yaml'),
        entity.EnvironmentVariable(name='foo', value='bar'),
        entity.EnvironmentVariable(name='ROS_LOG_DIR', value='/tmp'),
        entity.RosNode(name='talker', executable='chatter/talker'),
        entity.RosNode(name='listener', executable='chatter/listener'),
    ])
