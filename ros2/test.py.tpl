import os
import sys

import ros2cli.cli
import ros2test.command.test

import {ament_setup}

LAUNCH_FILE = '{launch_file}'

# The package name is intentionally undefined such that ros2test picks up
# the given launch file.
argv = [LAUNCH_FILE] + sys.argv[1:]

{ament_setup}.set_up_ament()

test_outputs_dir = os.environ.get('TEST_UNDECLARED_OUTPUTS_DIR')
if test_outputs_dir:
    os.environ['ROS_HOME'] = test_outputs_dir
    os.environ['ROS_LOG_DIR'] = test_outputs_dir

extension = ros2test.command.test.TestCommand()
sys.exit(ros2cli.cli.main(argv=argv, extension=extension))
