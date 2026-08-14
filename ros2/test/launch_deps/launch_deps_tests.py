import launch
import launch_pytest.tools
import pytest


@pytest.fixture
def child_launch_proc():
    return launch.actions.ExecuteProcess(
        cmd=['ros2/test/launch_deps/child_launch'],
        cached_output=True,
    )


@launch_pytest.fixture
def launch_description(child_launch_proc):
    return launch.LaunchDescription(
        [child_launch_proc,
         launch_pytest.actions.ReadyToTest()])


@pytest.mark.launch(fixture=launch_description)
def test_launch_deps(child_launch_proc, launch_context):
    launch_pytest.tools.wait_for_start_sync(launch_context,
                                            child_launch_proc,
                                            timeout=10)

    def check_output(output):
        assert any('Hello, count' in line for line in output.splitlines())

    launch_pytest.tools.assert_output_sync(launch_context,
                                           child_launch_proc,
                                           check_output,
                                           timeout=15)
