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

#include <memory>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageResizer final : public rclcpp::Node {
 public:
  ImageResizer()
      : Node("image_resizer"), sub_(), pub_(), scale_(1.0) {
    declare_parameter("scale", 1.0);
    scale_ = get_parameter("scale").as_double();

    declare_parameter("input.image_transport", "raw");
    const auto transport_hint = get_parameter("input.image_transport").as_string();

    sub_ = image_transport::create_subscription(
        this, "~/input",
        std::bind(&ImageResizer::imageCallback, this, std::placeholders::_1),
        transport_hint);
    pub_ = image_transport::create_publisher(this, "~/output");

    RCLCPP_INFO(get_logger(), "Image resizer started with scale: %.2f, transport_hint: %s",
                scale_, transport_hint.c_str());
  }

 private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    RCLCPP_INFO(get_logger(), "Received image %dx%d, resizing to scale %.2f",
                msg->width, msg->height, scale_);

    if (msg->encoding != sensor_msgs::image_encodings::BGR8) {
      RCLCPP_ERROR(get_logger(),
                   "Unsupported encoding '%s'. Only '%s' is supported.",
                   msg->encoding.c_str(),
                   sensor_msgs::image_encodings::BGR8);
      return;
    }

    auto resized_image = sensor_msgs::msg::Image();
    resized_image.header = msg->header;
    resized_image.encoding = msg->encoding;
    resized_image.is_bigendian = msg->is_bigendian;

    const size_t resized_width = static_cast<size_t>(msg->width * scale_);
    const size_t resized_height = static_cast<size_t>(msg->height * scale_);
    resized_image.width = resized_width;
    resized_image.height = resized_height;
    resized_image.step = resized_width * 3;
    resized_image.data.resize(resized_height * resized_width * 3);

    for (size_t y = 0; y < resized_height; ++y) {
      for (size_t x = 0; x < resized_width; ++x) {
        size_t src_x = static_cast<size_t>(x / scale_);
        size_t src_y = static_cast<size_t>(y / scale_);

        if (src_x >= msg->width) src_x = msg->width - 1;
        if (src_y >= msg->height) src_y = msg->height - 1;

        const size_t src_index = (src_y * msg->width + src_x) * 3;
        const size_t dst_index = (y * resized_width + x) * 3;

        resized_image.data[dst_index] = msg->data[src_index];
        resized_image.data[dst_index + 1] = msg->data[src_index + 1];
        resized_image.data[dst_index + 2] = msg->data[src_index + 2];
      }
    }

    pub_.publish(resized_image);
    RCLCPP_INFO(get_logger(), "Published resized image %dx%d",
                resized_image.width, resized_image.height);
  }

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
  double scale_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageResizer>());
  rclcpp::shutdown();
  return 0;
}
