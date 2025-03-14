// Copyright 2023 Foxglove Technologies Inc
// SPDX-License-Identifier: MIT
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "foxglove_bridge/ros2_foxglove_bridge.hpp"
#include "foxglove_bridge/test/test_client.hpp"
#include "foxglove_bridge/websocket_client.hpp"
#include "gtest/gtest.h"
#include "std_msgs/msg/string.hpp"
#include "websocketpp/config/asio_client.hpp"

constexpr char URI[] = "ws://localhost:8765";

// Binary representation of std_msgs/msg/String for "hello world"
constexpr uint8_t HELLO_WORLD_BINARY[] = {0,   1,   0,   0,   12,  0,   0,
                                          0,   104, 101, 108, 108, 111, 32,
                                          119, 111, 114, 108, 100, 0};

constexpr auto ONE_SECOND = std::chrono::seconds(1);
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(10);

TEST(SmokeTest, testConnection) {
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  EXPECT_EQ(std::future_status::ready,
            wsClient.connect(URI).wait_for(DEFAULT_TIMEOUT));
}

TEST(SmokeTest, testSubscription) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  std_msgs::msg::String rosMsg;
  rosMsg.data = "hello world";

  auto node = rclcpp::Node::make_shared("tester");
  rclcpp::QoS qos = rclcpp::QoS{rclcpp::KeepLast(1lu)};
  qos.reliable();
  qos.transient_local();
  auto pub = node->create_publisher<std_msgs::msg::String>(topic_name, qos);
  pub->publish(rosMsg);

  // Connect a few clients and make sure that they receive the correct message
  const auto clientCount = 3;
  for (auto i = 0; i < clientCount; ++i) {
    // Set up a client and subscribe to the channel.
    auto client =
        std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
    auto channelFuture = foxglove::waitForChannel(client, topic_name);
    ASSERT_EQ(std::future_status::ready,
              client->connect(URI).wait_for(ONE_SECOND));
    ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(ONE_SECOND));
    const foxglove::Channel channel = channelFuture.get();
    const foxglove::SubscriptionId subscriptionId = 1;

    // Subscribe to the channel and confirm that the promise resolves
    auto msgFuture = waitForChannelMsg(client.get(), subscriptionId);
    client->subscribe({{subscriptionId, channel.id}});
    ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(ONE_SECOND));
    const auto msgData = msgFuture.get();
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), msgData.size());
    EXPECT_EQ(0,
              std::memcmp(HELLO_WORLD_BINARY, msgData.data(), msgData.size()));

    // Unsubscribe from the channel again.
    client->unsubscribe({subscriptionId});
  }
}

TEST(SmokeTest, testSubscriptionParallel) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  std_msgs::msg::String rosMsg;
  rosMsg.data = "hello world";

  auto node = rclcpp::Node::make_shared("tester");
  rclcpp::QoS qos = rclcpp::QoS{rclcpp::KeepLast(1lu)};
  qos.reliable();
  qos.transient_local();
  auto pub = node->create_publisher<std_msgs::msg::String>(topic_name, qos);
  pub->publish(rosMsg);

  // Connect a few clients (in parallel) and make sure that they receive the
  // correct message
  const foxglove::SubscriptionId subscriptionId = 1;
  auto clients = {
      std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
      std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
      std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
  };

  std::vector<std::future<std::vector<uint8_t>>> futures;
  for (auto client : clients) {
    futures.push_back(waitForChannelMsg(client.get(), subscriptionId));
  }

  for (auto client : clients) {
    auto channelFuture = foxglove::waitForChannel(client, topic_name);
    ASSERT_EQ(std::future_status::ready,
              client->connect(URI).wait_for(ONE_SECOND));
    ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(ONE_SECOND));
    const foxglove::Channel channel = channelFuture.get();
    client->subscribe({{subscriptionId, channel.id}});
  }

  for (auto& future : futures) {
    ASSERT_EQ(std::future_status::ready, future.wait_for(DEFAULT_TIMEOUT));
    auto msgData = future.get();
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), msgData.size());
    EXPECT_EQ(0,
              std::memcmp(HELLO_WORLD_BINARY, msgData.data(), msgData.size()));
  }

  for (auto client : clients) {
    client->unsubscribe({subscriptionId});
  }
}

TEST(SmokeTest, testPublishing) {
  foxglove::ClientAdvertisement advertisement;
  advertisement.channelId = 1;
  advertisement.topic = "/foo";
  advertisement.encoding = "cdr";
  advertisement.schemaName = "std_msgs/String";

  // Set up a ROS node with a subscriber
  std::promise<std::string> msgPromise;
  auto msgFuture = msgPromise.get_future();
  auto node = rclcpp::Node::make_shared("tester");
  auto sub = node->create_subscription<std_msgs::msg::String>(
      advertisement.topic, 10,
      [&msgPromise](const std_msgs::msg::String::SharedPtr msg) {
        msgPromise.set_value(msg->data);
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Set up the client, advertise and publish the binary message
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  ASSERT_EQ(std::future_status::ready,
            wsClient.connect(URI).wait_for(DEFAULT_TIMEOUT));
  wsClient.advertise({advertisement});
  std::this_thread::sleep_for(ONE_SECOND);
  wsClient.publish(advertisement.channelId, HELLO_WORLD_BINARY,
                   sizeof(HELLO_WORLD_BINARY));
  wsClient.unadvertise({advertisement.channelId});

  // Ensure that we have received the correct message via our ROS subscriber
  const auto ret = executor.spin_until_future_complete(msgFuture, ONE_SECOND);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
  EXPECT_EQ("hello world", msgFuture.get());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const size_t numThreads = 2;
  auto executor = rclcpp::executors::MultiThreadedExecutor::make_shared(
      rclcpp::ExecutorOptions{}, numThreads);

  auto node = std::make_shared<foxglove_bridge::FoxgloveBridge>();
  executor->add_node(node);

  std::thread spinnerThread([&executor]() { executor->spin(); });

  const auto testResult = RUN_ALL_TESTS();
  executor->cancel();
  spinnerThread.join();
  rclcpp::shutdown();

  return testResult;
}
