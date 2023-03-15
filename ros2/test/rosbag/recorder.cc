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
#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "std_msgs/msg/string.hpp"

constexpr auto kTopicName = "topic";

class BagSplitEventListener : public rclcpp::Node {
 public:
  using WriteSplitEvent = rosbag2_interfaces::msg::WriteSplitEvent;

  BagSplitEventListener() : Node("bag_split_event_listener") {
    subscription_ = create_subscription<WriteSplitEvent>(
        "events/write_split", 10,
        [this](WriteSplitEvent::SharedPtr /*msg*/) { ++split_count_; });
  }

  auto split_count() const { return split_count_; }

 private:
  rclcpp::Subscription<WriteSplitEvent>::SharedPtr subscription_;
  int split_count_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  rosbag2_transport::RecordOptions record_options;
  record_options.topics = {kTopicName};

  if (record_options.rmw_serialization_format.empty()) {
    record_options.rmw_serialization_format =
        std::string(rmw_get_serialization_format());
  }

  auto writer =
      rosbag2_transport::ReaderWriterFactory::make_writer(record_options);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = std::string(std::getenv("TEST_TMPDIR")) + "/bag";
  storage_options.storage_id = std::getenv("STORAGE_ID");
  // We use the bag split event as a signal that we can stop.
  constexpr uint64_t kSqlite3MinimumSplitSize = 86016;
  storage_options.max_bagfile_size = kSqlite3MinimumSplitSize;
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer), storage_options, record_options);
  executor.add_node(recorder);

  auto split_counter = std::make_shared<BagSplitEventListener>();
  executor.add_node(split_counter);

  recorder->record();

  while (rclcpp::ok() && split_counter->split_count() == 0) {
    executor.spin_once(std::chrono::milliseconds(10));
  }

  return rclcpp::shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
