"""Test rclpy.init() in different test drivers.

What is tested here is that ROS_HOME (or ROS_LOG_DIR) is set.
If it is not set, rclpy.init() will fail to initialize.

It makes sense to test this for each test driver because each uses a different
wrapper to set the environment vars.
"""
import launch
import launch_testing.actions
import launch_testing.markers
import rclpy


def test_rclpy_init():
    """This will run when using ros2_test with pytest as test driver."""
    rclpy.init()


@launch_testing.markers.keep_alive
def generate_test_description():
    """This will run when using ros2_test with launch_testing as test driver."""
    rclpy.init()
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest(),
    ])


if __name__ == '__main__':
    """This will run when using ros2_py_test."""
    test_rclpy_init()
