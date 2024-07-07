import threading
import unittest

import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
import std_msgs.msg


@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable='ros2/test/rust/publisher',
            name='publisher',
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class Subscriber(rclpy.node.Node):

    def __init__(self):
        super().__init__('subscriber')

        self.sububscriptionubscription = self.create_subscription(
            std_msgs.msg.String, '/topic', self._on_topic, 10)
        self.messages = []
        self.messages_received = threading.Event()

    def _on_topic(self, msg):
        self.messages.append(msg)
        if len(self.messages) == 2:
            self.messages_received.set()


class TestRustPublisher(unittest.TestCase):

    def test_rust_publisher(self):
        rclpy.init()
        try:
            subscriber = Subscriber()
            thread = threading.Thread(target=lambda node: rclpy.spin(node),
                                      args=(subscriber,))
            thread.start()
            event_triggered = subscriber.messages_received.wait(timeout=10.0)
            self.assertTrue(event_triggered,
                            'timeout while waiting for messages')

            msgs = subscriber.messages
            self.assertGreaterEqual(len(msgs), 2)
            self.assertTrue(msgs[0].data.startswith('Hello, world!'))
        finally:
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestRustPublisherShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
