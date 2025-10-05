#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "publisher_parameters.hpp"

// This namespace will match the top-level entry in the .yamle file
namespace publisher {

class PublisherNode : public rclcpp::Node {
 public:
  PublisherNode() : Node("publisher_params_node") {
    param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
    param_listener_->setUserCallback(
        [this](const auto& params) { reconfigure_callback(params); }
        );
    params_ = param_listener_->get_params();
    print_params();
    int64_t publish_rate_ms = std::round(1000.0 / params_.publish_frequency_hz);

    publisher_ = create_publisher<std_msgs::msg::String>(params_.publish_topic, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(publish_rate_ms), [this]() { timer_callback(); });
  }

  void reconfigure_callback(const Params& params) {
    params_ = params;
    print_params();
  }

  void print_params() {
    std::stringstream names_ss;
    names_ss << "[";
    bool first = true;
    for (const auto& name : params_.names) {
      if (!first) {
        names_ss << ", ";
      }
      first = false;
      names_ss << name;
    }
    names_ss << "]";

    RCLCPP_INFO(get_logger(), "Received new parameters:");
    RCLCPP_INFO(get_logger(), "  names: %s", names_ss.str().c_str());
    RCLCPP_INFO(get_logger(), "  publish_frequency_hz: %f", params_.publish_frequency_hz);
    RCLCPP_INFO(get_logger(), "  publish_topic: %s", params_.publish_topic.c_str());
  }

  void timer_callback() {
    auto published_name = params_.names[name_idx];
    name_idx = (name_idx + 1) % params_.names.size();
    std_msgs::msg::String message;
    message.data = published_name;
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  size_t name_idx = 0;
};

} // namespace publisher

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<publisher::PublisherNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
