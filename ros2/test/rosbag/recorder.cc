// Copyright 2022 Milan Vukov
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
#include "rosbag2_transport/recorder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "std_msgs/msg/string.hpp"

constexpr auto kTopicName = "topic";

class MessageCounter : public rclcpp::Node {
 public:
  MessageCounter() : Node("message_counter") {
    subscription_ = create_subscription<std_msgs::msg::String>(
        kTopicName, 10,
        [this](std_msgs::msg::String::SharedPtr /*msg*/) { ++msg_count_; });
  }

  auto msg_count() const { return msg_count_; }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int msg_count_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto msg_counter = std::make_shared<MessageCounter>();
  executor.add_node(msg_counter);

  rosbag2_transport::RecordOptions record_options;
  record_options.topics = {kTopicName};

  if (record_options.rmw_serialization_format.empty()) {
    record_options.rmw_serialization_format =
        std::string(rmw_get_serialization_format());
  }

  auto writer =
      rosbag2_transport::ReaderWriterFactory::make_writer(record_options);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri =
      std::string(std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) + "/bag";
  storage_options.storage_id = "sqlite3";
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer), storage_options, record_options);
  recorder->record();
  executor.add_node(recorder);

  while (rclcpp::ok()) {
    executor.spin_once(std::chrono::milliseconds(10));
    if (msg_counter->msg_count() > 10) {
      break;
    }
  }

  return rclcpp::shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
