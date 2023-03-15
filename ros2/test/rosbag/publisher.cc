// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Publisher : public rclcpp::Node {
 public:
  Publisher() : Node("minimal_publisher") {
    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      // Produce reasonably large messages so we quickly reach the size limit
      // that causes the recorder to split the bag (we use this as a signal that
      // the recorder has received a sufficient number of messages).
      constexpr std::size_t kMinimumMessageSize = 8192;
      message.data = std::string(kMinimumMessageSize, 'x');
      publisher_->publish(message);
    };
    timer_ = create_wall_timer(std::chrono::milliseconds(20), timer_callback);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
