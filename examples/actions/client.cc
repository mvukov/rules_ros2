// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fibonacci_msgs/action/fibonacci.hpp"

class MinimalActionClient : public rclcpp::Node {
 public:
  using Fibonacci = fibonacci_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit MinimalActionClient(
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
      : Node("minimal_action_client", node_options), goal_done_(false) {
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(
        get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "fibonacci");

    timer_ =
        create_wall_timer(std::chrono::milliseconds(500),
                          std::bind(&MinimalActionClient::send_goal, this));
  }

  bool is_goal_done() const { return goal_done_; }

  void send_goal() {
    using namespace std::placeholders;  // NOLINT

    timer_->cancel();

    goal_done_ = false;

    if (!client_ptr_) {
      RCLCPP_ERROR(get_logger(), "Action client not initialized");
    }

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      goal_done_ = true;
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MinimalActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MinimalActionClient::result_callback, this, _1);
    auto goal_handle_future =
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(
      const GoalHandleFibonacci::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleFibonacci::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> feedback) {
    RCLCPP_INFO(get_logger(), "Next number in sequence received: %" PRId32,
                feedback->sequence.back());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult& result) {
    goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(get_logger(), "Result received");
    for (auto number : result.result->sequence) {
      RCLCPP_INFO(get_logger(), "%" PRId32, number);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MinimalActionClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}
