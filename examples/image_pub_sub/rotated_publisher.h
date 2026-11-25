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

#ifndef ROTATED_PUBLISHER_H
#define ROTATED_PUBLISHER_H

#include <string>

#include "image_transport/simple_publisher_plugin.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_pub_sub {

class RotatedPublisher final
    : public image_transport::SimplePublisherPlugin<sensor_msgs::msg::Image> {
 public:
  virtual std::string getTransportName() const final { return "rotated"; }

 protected:
  virtual void publish(const sensor_msgs::msg::Image& message,
                       const PublishFn& publish_fn) const final;
};

}  // namespace image_pub_sub

#endif  // ROTATED_PUBLISHER_H
