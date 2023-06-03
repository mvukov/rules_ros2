// Copyright 2023 Milan Vukov
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
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class Publisher : public rclcpp::Node {
 public:
  Publisher() : Node("publisher") {
    declare_parameter("callback_period_ms", 50);
    auto callback_period_ms = get_parameter("callback_period_ms").as_int();

    delta_angle_ = 2. * M_PI * 1e-3 * callback_period_ms / 5.;

    publisher_ =
        create_publisher<geometry_msgs::msg::PointStamped>("point", 10);
    auto timer_callback = [this]() -> void {
      msg_.header.stamp = get_clock()->now();
      angle_ += delta_angle_;
      msg_.point.x = std::sin(angle_);
      msg_.point.y = std::cos(angle_);
      publisher_->publish(msg_);
    };
    timer_ = create_wall_timer(std::chrono::milliseconds(callback_period_ms),
                               timer_callback);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  geometry_msgs::msg::PointStamped msg_;
  double delta_angle_ = 0.;
  double angle_ = 0.;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
