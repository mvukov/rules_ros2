// Copyright 2016 Open Source Robotics Foundation, Inc.
// Copyright 2022 wayve.ai
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

#include <limits>
#include <memory>

#include "chatter_interface/msg/chatter.hpp"
#include "rclcpp/rclcpp.hpp"

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = create_subscription<chatter_interface::msg::Chatter>(
        "topic", 10, [this](chatter_interface::msg::Chatter::SharedPtr msg) {
          static uint64_t prev_count{std::numeric_limits<uint64_t>::max()};
          auto current_timestamp =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now().time_since_epoch())
                  .count();
          std::string str{msg->data.begin(),
                          msg->data.begin() + msg->data_length};
          RCLCPP_INFO(get_logger(), "Delay %lu us, I heard: '%s'",
                      current_timestamp - msg->timestamp, str.c_str());
          if ((prev_count != std::numeric_limits<uint64_t>::max()) &&
              (msg->count != prev_count + 1)) {
            RCLCPP_WARN(get_logger(), "Some messages are missed!");
          }
          prev_count = msg->count;
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
