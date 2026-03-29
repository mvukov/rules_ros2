from ros2.houston import entity


def create_deployment() -> entity.Deployment:
    return entity.Deployment([
        entity.ParametersFile(path='chatter/override_params.yaml'),
    ])
