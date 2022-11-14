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

#include <chrono>
#include <cstring>
#include <memory>

#include "chatter_interface/msg/chatter.hpp"
#include "rclcpp/rclcpp.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    declare_parameter("callback_period_ms", 500);
    auto callback_period_ms = get_parameter("callback_period_ms").as_int();

    publisher_ = create_publisher<chatter_interface::msg::Chatter>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = publisher_->borrow_loaned_message();
      std::string str = "Hello, world! " + std::to_string(count_++);
      auto copy_size = str.size() < message.get().data.size()
                           ? str.size()
                           : message.get().data.size();
      std::memcpy(message.get().data.begin(), str.c_str(), copy_size);
      message.get().data_length = copy_size;
      message.get().timestamp =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      message.get().count = count_;
      RCLCPP_INFO(get_logger(), "Publishing: '%s'", str.c_str());
      publisher_->publish(std::move(message));
    };
    timer_ = create_wall_timer(std::chrono::milliseconds(callback_period_ms),
                               timer_callback);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<chatter_interface::msg::Chatter>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
