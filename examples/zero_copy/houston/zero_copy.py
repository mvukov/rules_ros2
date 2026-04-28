from ros2.houston import entity


def create_deployment() -> entity.Deployment:
    return entity.Deployment([
        entity.EnvironmentVariable(name='ROS_DISABLE_LOANED_MESSAGES',
                                   value='0'),
        entity.EnvironmentVariable(name='CYCLONEDDS_URI',
                                   value='zero_copy/cyclonedds.xml'),
        entity.Process(
            name='iceoryx_roudi',
            executable='../iceoryx/iceoryx/bin/iox-roudi',
            arguments=['-c', 'zero_copy/roudi.toml'],
        ),
        entity.RosNode(name='talker', executable='zero_copy/talker'),
        entity.RosNode(name='listener', executable='zero_copy/listener'),
        entity.ParametersFile(path='zero_copy/houston/zero_copy_params.yaml')
    ])
