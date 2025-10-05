import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from gen_parameters.publisher_parameters import publisher as publisher_parameters 


class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.param_listener = publisher_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.param_listener.set_user_callback(self.param_callback)
        self.print_params()

        self.name_idx = 0
        self.publisher = self.create_publisher(String, self.params.publish_topic, 10)
        publish_rate_ms = 1.0 / self.params.publish_frequency_hz
        self.timer = self.create_timer(publish_rate_ms, self.timer_callback)

    def timer_callback(self):
        name = self.params.names[self.name_idx]
        self.name_idx = (self.name_idx + 1) % len(self.params.names)
        msg = String()
        msg.data = name
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def param_callback(self, params):
        self.params = self.param_listener.get_params()
        self.print_params()

    def print_params(self):
        self.get_logger().info(f"Current parameters:")
        self.get_logger().info(f"  names: {self.params.names}")
        self.get_logger().info(f"  publish_topic: {self.params.publish_topic}")
        self.get_logger().info(f"  publish_frequency_hz: {self.params.publish_frequency_hz}")

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
