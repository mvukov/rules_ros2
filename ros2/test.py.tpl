import contextlib
import os
import sys

import domain_coordinator
import launch_testing.launch_test
import launch_testing_ros

{ament_setup}

LAUNCH_FILE = '{launch_file}'

# The package name is intentionally undefined such that ros2test picks up
# the given launch file.
sys.argv = sys.argv[:1] + [
    f'--junit-xml={os.environ["XML_OUTPUT_FILE"]}',
    LAUNCH_FILE,
] + sys.argv[1:]

if 'ROS_HOME' not in os.environ and 'ROS_LOG_DIR' not in os.environ:
    ros_output_dir = os.environ.get('TEST_UNDECLARED_OUTPUTS_DIR')
    if ros_output_dir is None:
        ros_output_dir = os.environ.get('TEST_TMPDIR')
    if ros_output_dir is not None:
        os.environ['ROS_HOME'] = ros_output_dir

with contextlib.ExitStack() as stack:
    if 'ROS_DOMAIN_ID' not in os.environ:
        domain_id = stack.enter_context(domain_coordinator.domain_id())
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    print(f'Running with ROS_DOMAIN_ID {os.environ["ROS_DOMAIN_ID"]}')

    parser, args = launch_testing.launch_test.parse_arguments()
    exit_code = launch_testing.launch_test.run(
        parser, args, test_runner_cls=launch_testing_ros.LaunchTestRunner)
    sys.exit(exit_code)
