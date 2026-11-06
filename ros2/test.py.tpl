import os
import sys

import launch_testing.launch_test
import launch_testing_ros

LAUNCH_FILE = '{launch_file}'

# The package name is intentionally undefined such that
# launch_testing.launch_test picks up the given launch file.
sys.argv = sys.argv[:1] + [
    f'--junit-xml={os.environ["XML_OUTPUT_FILE"]}',
    LAUNCH_FILE,
] + sys.argv[1:]

parser, args = launch_testing.launch_test.parse_arguments()
exit_code = launch_testing.launch_test.run(
    parser, args, test_runner_cls=launch_testing_ros.LaunchTestRunner)
sys.exit(exit_code)
