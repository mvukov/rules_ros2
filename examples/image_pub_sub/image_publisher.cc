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

#include <chrono>
#include <memory>
#include <vector>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePublisher final : public rclcpp::Node {
 public:
  ImagePublisher()
      : Node("image_publisher"), pub_(), test_image_(), timer_(nullptr) {
    declare_parameter("width", 1280);
    declare_parameter("height", 720);
    declare_parameter("frequency", 5.0);

    const auto width = get_parameter("width").as_int();
    const auto height = get_parameter("height").as_int();
    const auto frequency = get_parameter("frequency").as_double();

    pub_ = image_transport::create_publisher(this, "~/output");

    test_image_.header.frame_id = "camera";
    test_image_.height = height;
    test_image_.width = width;
    test_image_.encoding = sensor_msgs::image_encodings::BGR8;
    test_image_.is_bigendian = false;
    test_image_.step = width * 3;
    test_image_.data.resize(height * width * 3);

    const std::vector<std::vector<uint8_t>> colors = {
        {128, 128, 128},  // Gray
        {0, 255, 255},    // Yellow
        {255, 255, 0},    // Cyan
        {0, 255, 0},      // Green
        {255, 0, 255},    // Magenta
        {0, 0, 255},      // Red
        {255, 0, 0},      // Blue
    };

    for (size_t y = 0; y < static_cast<size_t>(height); ++y) {
      for (size_t x = 0; x < static_cast<size_t>(width); ++x) {
        size_t bar_index = x / (width / colors.size());
        if (bar_index >= colors.size()) {
          bar_index = colors.size() - 1;
        }
        const size_t pixel_index = (y * width + x) * 3;
        test_image_.data[pixel_index] = colors[bar_index][0];      // B
        test_image_.data[pixel_index + 1] = colors[bar_index][1];  // G
        test_image_.data[pixel_index + 2] = colors[bar_index][2];  // R
      }
    }

    const auto timer_callback = [this]() -> void {
      test_image_.header.stamp = now();
      RCLCPP_INFO(get_logger(), "Publishing image %dx%d", test_image_.width,
                  test_image_.height);
      pub_.publish(test_image_);
    };

    const auto period_ms = static_cast<int>(1000.0 / frequency);
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               timer_callback);
  }

 private:
  image_transport::Publisher pub_;
  sensor_msgs::msg::Image test_image_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
