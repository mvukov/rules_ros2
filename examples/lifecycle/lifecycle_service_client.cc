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
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;  // NOLINT
using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;

// which node to handle
static constexpr auto kLifecycleNodeName = "lc_talker";

// Every lifecycle node has various services
// attached to it. By convention, we use the format of
// <node name>/<service name>.
// In this demo, we use get_state and change_state
// and thus the two service topics are:
// lc_talker/get_state
// lc_talker/change_state
static constexpr auto kNodeGetStateTopic = "lc_talker/get_state";
static constexpr auto kNodeChangeStateTopic = "lc_talker/change_state";

template <typename FutureT, typename WaitTimeT>
std::future_status WaitForResult(FutureT& future,  // NOLINT
                                 WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status =
        future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleServiceClient : public rclcpp::Node {
 public:
  explicit LifecycleServiceClient(const std::string& node_name)
      : Node(node_name) {
    // Every lifecycle node spawns automatically a couple
    // of services which allow an external interaction with
    // these nodes.
    // The two main important ones are GetState and ChangeState.
    client_get_state_ =
        create_client<lifecycle_msgs::srv::GetState>(kNodeGetStateTopic);
    client_change_state_ =
        create_client<lifecycle_msgs::srv::ChangeState>(kNodeChangeStateTopic);
  }

  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  auto GetState(std::chrono::seconds time_out = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                   client_get_state_->get_service_name());
      return State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result =
        client_get_state_->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = WaitForResult(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(),
                   "Server time out while getting current state for node %s",
                   kLifecycleNodeName);
      return State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer. So let's print the current state.
    if (future_result.get()) {
      RCLCPP_INFO(get_logger(), "Node %s has current state %s.",
                  kLifecycleNodeName,
                  future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s",
                   kLifecycleNodeName);
      return State::PRIMARY_STATE_UNKNOWN;
    }
  }

  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  bool ChangeState(std::uint8_t transition,
                   std::chrono::seconds time_out = 3s) {
    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                   client_change_state_->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result =
        client_change_state_->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = WaitForResult(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(),
                   "Server time out while getting current state for node %s",
                   kLifecycleNodeName);
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.",
                  static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Failed to trigger transition %u",
                  static_cast<unsigned int>(transition));
      return false;
    }
  }

 private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>
      client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
      client_change_state_;
};

/**
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
void TriggerLifecycle(std::shared_ptr<LifecycleServiceClient> lc_client) {
  rclcpp::WallRate time_between_state_changes(0.5);  // 2s

  auto execute_transition = [&time_between_state_changes, lc_client](
                                auto transition, bool wait) {
    if (wait) {
      time_between_state_changes.sleep();
    }
    if (!lc_client->ChangeState(transition)) {
      return false;
    }
    if (lc_client->GetState() == State::PRIMARY_STATE_UNKNOWN) {
      return false;
    }
    return true;
  };

  // configure
  if (!execute_transition(Transition::TRANSITION_CONFIGURE, false)) {
    return;
  }

  // activate
  if (!execute_transition(Transition::TRANSITION_ACTIVATE, true)) {
    return;
  }

  // deactivate
  if (!execute_transition(Transition::TRANSITION_DEACTIVATE, true)) {
    return;
  }

  // activate it again
  if (!execute_transition(Transition::TRANSITION_ACTIVATE, true)) {
    return;
  }

  // and deactivate it again
  if (!execute_transition(Transition::TRANSITION_DEACTIVATE, true)) {
    return;
  }

  // we cleanup
  if (!execute_transition(Transition::TRANSITION_CLEANUP, true)) {
    return;
  }

  // and finally shutdown
  // Note: We have to be precise here on which shutdown transition id to call
  // We are currently in the unconfigured state and thus have to call
  // TRANSITION_UNCONFIGURED_SHUTDOWN
  if (!execute_transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, true)) {
    return;
  }
}

void WakeExecutor(std::shared_future<void> future,
                  rclcpp::executors::SingleThreadedExecutor& exec) {  // NOLINT
  future.wait();
  // Wake the executor when the script is done
  // https://github.com/ros2/rclcpp/issues/1916
  exec.cancel();
}

int main(int argc, char** argv) {
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lc_client);

  std::shared_future<void> script =
      std::async(std::launch::async, std::bind(TriggerLifecycle, lc_client));
  auto wake_exec = std::async(
      std::launch::async, std::bind(WakeExecutor, script, std::ref(executor)));

  executor.spin_until_future_complete(script);
  rclcpp::shutdown();
  return 0;
}
