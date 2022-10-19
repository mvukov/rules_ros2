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

#include <iostream>
#include <memory>

#include "chatter_interface/msg/chatter.hpp"
#include "rclcpp/rclcpp.hpp"

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = create_subscription<chatter_interface::msg::Chatter>(
        "topic", 10, [this](chatter_interface::msg::Chatter::SharedPtr msg) {
          auto current_timestamp =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
          RCLCPP_INFO(get_logger(), "data length %lu", msg->data_length);
          std::string str{msg->data.begin(), msg->data.begin() + msg->data_length};
          RCLCPP_INFO(get_logger(), "I heard: '%s', delay %lu us", str.c_str(), current_timestamp - msg->timestamp);
        });
  }

 private:
  rclcpp::Subscription<chatter_interface::msg::Chatter>::SharedPtr
      subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
