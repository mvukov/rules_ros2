"""Entry point of `ros2_test` targets that use launch_testing.

argv[1] is the launch file (an absolute path, resolved by the launcher), the
remaining arguments are forwarded to launch_test.
"""
import os
import sys

import launch_testing.launch_test
import launch_testing_ros


def main() -> int:
    # The package name is intentionally undefined such that
    # launch_testing.launch_test picks up the given launch file.
    launch_file = sys.argv[1]
    sys.argv = [
        sys.argv[0],
        f'--junit-xml={os.environ["XML_OUTPUT_FILE"]}',
        launch_file,
        *sys.argv[2:],
    ]

    parser, args = launch_testing.launch_test.parse_arguments()
    return launch_testing.launch_test.run(
        parser, args, test_runner_cls=launch_testing_ros.LaunchTestRunner)


if __name__ == '__main__':
    sys.exit(main())
