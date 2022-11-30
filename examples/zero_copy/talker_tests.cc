// Copyright 2022 Milan Vukov
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

#include "chatter_interface/msg/chatter.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace zero_copy::testing {

using ::testing::Ge;

class TalkerTester : public rclcpp::Node {
 public:
  TalkerTester() : Node("talker_tester") {
    subscription_ = create_subscription<chatter_interface::msg::Chatter>(
        "topic", 10,
        [this](chatter_interface::msg::Chatter::SharedPtr /*msg*/) {
          ++msg_count_;
        });
  }

  auto msg_count() const { return msg_count_; }

 private:
  rclcpp::Subscription<chatter_interface::msg::Chatter>::SharedPtr
      subscription_;
  int msg_count_ = 0;
};

TEST(TestTalker, WhenTalkerPublishes_ExpectAtLeastTwoMsgsOnTopic) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<TalkerTester>();
  executor.add_node(node);
  while (rclcpp::ok()) {
    executor.spin_once(std::chrono::milliseconds(10));
    if (node->msg_count() > 2) {
      break;
    }
  }
  EXPECT_THAT(node->msg_count(), Ge(2));
}

}  // namespace zero_copy::testing

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  if (!rclcpp::shutdown() || result) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
