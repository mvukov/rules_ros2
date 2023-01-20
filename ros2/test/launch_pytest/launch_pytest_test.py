import sys

import launch
import launch_pytest.tools
import pytest


@pytest.fixture
def hello_proc():
    return launch.actions.ExecuteProcess(
        cmd=['ros2/test/launch_pytest/hello'],
        cached_output=True,
    )


@launch_pytest.fixture
def launch_description(hello_proc):
    return launch.LaunchDescription(
        [hello_proc, launch_pytest.actions.ReadyToTest()])


@pytest.mark.launch(fixture=launch_description)
def test_check_output(hello_proc, launch_context):
    launch_pytest.tools.wait_for_start_sync(launch_context,
                                            hello_proc,
                                            timeout=10)

    def check_output(output):
        assert output.splitlines() == [f'Hello, count {i}' for i in range(5)]

    launch_pytest.tools.assert_output_sync(launch_context,
                                           hello_proc,
                                           check_output,
                                           timeout=10)


if __name__ == '__main__':
    sys.exit(
        pytest.main(['-rP', '-p', 'launch_pytest.plugin', '--', sys.argv[0]]))
