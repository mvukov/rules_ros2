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

#include "rotated_publisher.h"

#include <vector>

#include "rclcpp/logging.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace image_pub_sub {

void RotatedPublisher::publish(const sensor_msgs::msg::Image& message,
                                const PublishFn& publish_fn) const {

  if (message.encoding != sensor_msgs::image_encodings::BGR8) {
    auto logger = rclcpp::get_logger("rotated_publisher");
    RCLCPP_ERROR(logger,
                 "RotatedPublisher only supports '%s' encoding, got '%s'",
                 sensor_msgs::image_encodings::BGR8,
                 message.encoding.c_str());
    return;
  }

  // Create rotated image by rotating +90 degrees (clockwise).
  // Rotate +90 deg clockwise: pixel at (x, y) moves to (height - 1 - y, x).
  sensor_msgs::msg::Image rotated_image = message;

  const size_t src_width = message.width;
  const size_t src_height = message.height;
  const size_t bytes_per_pixel = 3;

  rotated_image.width = src_height;
  rotated_image.height = src_width;
  rotated_image.step = rotated_image.width * bytes_per_pixel;

  std::vector<uint8_t> rotated_data(rotated_image.height * rotated_image.step);

  for (size_t src_y = 0; src_y < src_height; ++src_y) {
    for (size_t src_x = 0; src_x < src_width; ++src_x) {
      const size_t dst_x = src_height - 1 - src_y;
      const size_t dst_y = src_x;

      const size_t src_offset = (src_y * message.step) + (src_x * bytes_per_pixel);
      const size_t dst_offset = (dst_y * rotated_image.step) + (dst_x * bytes_per_pixel);

      rotated_data[dst_offset + 0] = message.data[src_offset + 0];
      rotated_data[dst_offset + 1] = message.data[src_offset + 1];
      rotated_data[dst_offset + 2] = message.data[src_offset + 2];
    }
  }

  rotated_image.data = rotated_data;
  publish_fn(rotated_image);
}

}  // namespace image_pub_sub

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(image_pub_sub::RotatedPublisher, image_transport::PublisherPlugin)
