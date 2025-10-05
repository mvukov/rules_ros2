#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "publisher_parameters.hpp"

class Publisher : public rclcpp::Node {
 public:
  Publisher() : Node("publisher_params_node") {
    publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      // Produce reasonably large messages so we quickly reach the size limit
      // that causes the recorder to split the bag (we use this as a signal that
      // the recorder has received a sufficient number of messages).
      constexpr std::size_t kMinimumMessageSize = 8192;
      message.data = std::string(kMinimumMessageSize, 'x');
      publisher_->publish(message);
    };
    timer_ = create_wall_timer(std::chrono::milliseconds(20), timer_callback);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
