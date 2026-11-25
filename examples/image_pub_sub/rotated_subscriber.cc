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

#include "rotated_subscriber.h"

#include <memory>
#include <vector>

#include "sensor_msgs/image_encodings.hpp"

namespace image_pub_sub {

void RotatedSubscriber::internalCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& message,
    const Callback& user_cb) {

  if (message->encoding != sensor_msgs::image_encodings::BGR8) {
    auto logger = rclcpp::get_logger("rotated_subscriber");
    RCLCPP_ERROR(logger,
                 "RotatedSubscriber only supports '%s' encoding, got '%s'",
                 sensor_msgs::image_encodings::BGR8,
                 message->encoding.c_str());
    return;
  }

  // The rotated image was already processed by the publisher (+90 deg).
  // For the subscriber, we need to rotate it -90 deg to restore the original:
  // pixel at (x, y) moves to (y, width - 1 - x).
  auto restored_image = std::make_shared<sensor_msgs::msg::Image>(*message);

  const size_t src_width = message->width;
  const size_t src_height = message->height;
  const size_t bytes_per_pixel = 3;

  restored_image->width = src_height;
  restored_image->height = src_width;
  restored_image->step = restored_image->width * bytes_per_pixel;

  std::vector<uint8_t> restored_data(restored_image->height * restored_image->step);

  for (size_t src_y = 0; src_y < src_height; ++src_y) {
    for (size_t src_x = 0; src_x < src_width; ++src_x) {
      const size_t dst_x = src_y;
      const size_t dst_y = src_width - 1 - src_x;

      const size_t src_offset = (src_y * message->step) + (src_x * bytes_per_pixel);
      const size_t dst_offset = (dst_y * restored_image->step) + (dst_x * bytes_per_pixel);

      restored_data[dst_offset + 0] = message->data[src_offset + 0];
      restored_data[dst_offset + 1] = message->data[src_offset + 1];
      restored_data[dst_offset + 2] = message->data[src_offset + 2];
    }
  }

  restored_image->data = restored_data;

  user_cb(restored_image);
}

}  // namespace image_pub_sub

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(image_pub_sub::RotatedSubscriber, image_transport::SubscriberPlugin)
