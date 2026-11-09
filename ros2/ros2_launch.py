"""Entry point of `ros2_launch` targets.

argv[1] is the launch file (an absolute path, resolved by the launcher), the
remaining arguments are forwarded to `ros2 launch`.
"""
import sys

from ros2cli import cli
from ros2launch.command import launch


def main() -> int:
    # The package name is intentionally undefined such that ros2launch picks up
    # the given launch file.
    extension = launch.LaunchCommand()
    return cli.main(argv=sys.argv[1:], extension=extension)


if __name__ == '__main__':
    sys.exit(main())
