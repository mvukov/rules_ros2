import sys

from ros2cli import cli
from ros2test.command import test

LAUNCH_FILE = '{launch_file}'

# The package name is intentionally undefined such that ros2test picks up
# the given launch file.
argv = [LAUNCH_FILE] + sys.argv[1:]

extension = test.TestCommand()
sys.exit(cli.main(argv=argv, extension=extension))
