// Copyright 2026 Milan Vukov
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

#include <memory>

#include "chatter_interfaces/msg/chatter_message.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * C++ listener node using proto2ros generated messages.
 */
class ProtoChatterListener : public rclcpp::Node {
 public:
  ProtoChatterListener() : Node("proto_chatter_listener") {
    subscription_ =
        create_subscription<chatter_interfaces::msg::ChatterMessage>(
            "chatter", 10,
            [this](chatter_interfaces::msg::ChatterMessage::UniquePtr msg) {
              RCLCPP_INFO(get_logger(),
                          "I heard: data='%s', timestamp=%ld",
                          msg->data.c_str(), msg->timestamp);
            });
  }

 private:
  rclcpp::Subscription<chatter_interfaces::msg::ChatterMessage>::SharedPtr
      subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProtoChatterListener>());
  rclcpp::shutdown();
  return 0;
}
