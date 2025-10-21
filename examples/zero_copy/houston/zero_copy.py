from ros2.houston import entity


def create_deployment() -> entity.Deployment:
    return entity.Deployment([
        entity.EnvironmentVariable(name='ROS_LOG_DIR', value='/tmp'),
        entity.EnvironmentVariable(name='ROS_DISABLE_LOANED_MESSAGES',
                                   value='0'),
        entity.EnvironmentVariable(name='CYCLONEDDS_URI',
                                   value='zero_copy/cyclonedds.xml'),
        entity.Process(
            name='iceoryx_roudi',
            executable='../iceoryx/iceoryx/bin/iox-roudi',
            arguments=['-c', 'zero_copy/roudi.toml'],
        ),
        entity.RosNode(name='talker',
                       executable='zero_copy/talker',
                       parameters_file='zero_copy/houston/talker.yaml'),
        entity.RosNode(name='listener', executable='zero_copy/listener'),
    ])
