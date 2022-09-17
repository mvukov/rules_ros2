import os
import sys

from ros2cli import cli
from ros2test.command import test

LAUNCH_FILE = '{launch_file}'

# The package name is intentionally undefined such that ros2test picks up
# the given launch file.
argv = [LAUNCH_FILE] + sys.argv[1:]

test_outputs_dir = os.environ.get('TEST_UNDECLARED_OUTPUTS_DIR')
if test_outputs_dir:
    os.environ['ROS_HOME'] = test_outputs_dir
    os.environ['ROS_LOG_DIR'] = test_outputs_dir

extension = test.TestCommand()
sys.exit(cli.main(argv=argv, extension=extension))
