# Copyright 2026 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Python talker node using proto2ros generated messages."""

import time

import rclpy
from chatter_interfaces.msg import ChatterMessage
from rclpy import node


class ProtoChatterTalker(node.Node):
    """Publishes ChatterMessage messages on the 'chatter' topic."""

    def __init__(self):
        super().__init__("proto_chatter_talker")
        self.publisher_ = self.create_publisher(ChatterMessage, "chatter", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """Publish a ChatterMessage with data and timestamp."""
        msg = ChatterMessage()
        msg.data = f"Hello from proto2ros: {self.i}"
        msg.timestamp = int(time.time() * 1000)  # milliseconds since epoch
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: data="{msg.data}", timestamp={msg.timestamp}'
        )
        self.i += 1


def main():
    rclpy.init()

    talker = ProtoChatterTalker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
