// Copyright 2026 Yuki Furuta
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

#ifndef ROTATED_SUBSCRIBER_H
#define ROTATED_SUBSCRIBER_H

#include <string>

#include "image_transport/simple_subscriber_plugin.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_pub_sub {

class RotatedSubscriber final
    : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::Image> {
 public:
  virtual ~RotatedSubscriber() {}

  virtual std::string getTransportName() const final { return "rotated"; }

 protected:
  virtual void internalCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& message,
      const Callback& user_cb) final;

  // Override 5-argument subscribeImpl to use SimpleSubscriberPlugin's implementation
  virtual void subscribeImpl(
      rclcpp::Node* node,
      const std::string& base_topic,
      const Callback& callback,
      rmw_qos_profile_t custom_qos,
      rclcpp::SubscriptionOptions options) override final {
    subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
  }
};

}  // namespace image_pub_sub

#endif  // ROTATED_SUBSCRIBER_H
