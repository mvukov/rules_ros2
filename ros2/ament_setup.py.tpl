import os

AMENT_PREFIX_PATH = 'AMENT_PREFIX_PATH'

def set_up_ament():
    ament_prefix_path = {ament_prefix_path}
    if ament_prefix_path is None:
        os.unsetenv(AMENT_PREFIX_PATH)
    else:
        os.environ[AMENT_PREFIX_PATH] = ament_prefix_path
