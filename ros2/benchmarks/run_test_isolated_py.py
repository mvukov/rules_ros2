"""A Python equivalent of //ros2/domain_coordinator:run_test_isolated.

Upstream's run_test_isolated.py from ament_cmake_ros, without the
ament_cmake_test dependency: reserves a ROS domain ID with upstream's
domain_coordinator, runs the command and returns its exit code.
"""
import os
import subprocess
import sys

import domain_coordinator


def main():
    command = sys.argv[1:]
    if not command:
        sys.exit('Usage: run_test_isolated_py <command> [<args>...]')
    if 'ROS_DOMAIN_ID' in os.environ or 'DISABLE_ROS_ISOLATION' in os.environ:
        os.execv(command[0], command)
    with domain_coordinator.domain_id() as domain_id:
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        return subprocess.call(command)


if __name__ == '__main__':
    sys.exit(main())
