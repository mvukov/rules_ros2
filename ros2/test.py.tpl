import os
import sys
import zlib

import launch_testing.launch_test
import launch_testing_ros

{ament_setup}

LAUNCH_FILE = '{launch_file}'

# The package name is intentionally undefined such that
# launch_testing.launch_test picks up the given launch file.
sys.argv = sys.argv[:1] + [
    f'--junit-xml={os.environ["XML_OUTPUT_FILE"]}',
    LAUNCH_FILE,
] + sys.argv[1:]

bazel_test_output_dir = os.environ.get('TEST_UNDECLARED_OUTPUTS_DIR')
if bazel_test_output_dir is None:
    bazel_test_output_dir = os.environ.get('TEST_TMPDIR')
if 'ROS_HOME' not in os.environ:
    os.environ['ROS_HOME'] = bazel_test_output_dir
if 'ROS_LOG_DIR' not in os.environ:
    os.environ['ROS_LOG_DIR'] = bazel_test_output_dir
if 'ROS_DOMAIN_ID' not in os.environ:
    # Bazel sandbox network isolation uses user namespaces via linux-sandbox,
    # which is typically not available in a Docker container, where Bazel falls
    # back to processwrapper-sandbox. In this case concurrently-running tests
    # can see each others DDS messages. Set the DOMAIN_ID explicitly to avoid
    # this (valid domain IDs are in the range 0..232).
    domain_seed = os.environ.get('TEST_TMPDIR', '') + os.environ.get('TEST_RANDOM_SEED', '')
    os.environ['ROS_DOMAIN_ID'] = str(zlib.crc32(domain_seed.encode()) % 232)

parser, args = launch_testing.launch_test.parse_arguments()
exit_code = launch_testing.launch_test.run(
    parser, args, test_runner_cls=launch_testing_ros.LaunchTestRunner)
sys.exit(exit_code)
