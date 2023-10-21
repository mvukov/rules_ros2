// Copyright 2023 Dennis Bruggner
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

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "rclcpp/rclcpp.hpp"

class Publisher : public rclcpp::Node {
 public:
  Publisher() : Node("heartbeat_publisher"), diagnostic_updater_{this} {
    diagnostic_updater_.setHardwareID("none");
    diagnostic_updater_.add("", &diagnostic_heartbeat_,
                            &diagnostic_updater::Heartbeat::run);
  }

 private:
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::Heartbeat diagnostic_heartbeat_{};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();  // will return false if already shutdown due to SIGINT,
                       // so ignore return value
  return EXIT_SUCCESS;
}
