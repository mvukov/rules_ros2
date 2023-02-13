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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "image_transport/publisher_plugin.hpp"
#include "image_transport/subscriber_plugin.hpp"
#include "pluginlib/class_loader.hpp"

using ::testing::Eq;

TEST(TestImageTransportPlugins,
     WhenRawPluginsAvailable_EnsurePublisherPluginCanBeLoaded) {
  pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader(
      "image_transport", "image_transport::PublisherPlugin");

  std::shared_ptr<image_transport::PublisherPlugin> pub =
      pub_loader.createSharedInstance("image_transport/raw_pub");
  EXPECT_THAT(pub->getTransportName(), Eq("raw"));
}

TEST(TestImageTransportPlugins,
     WhenRawPluginsAvailable_EnsureSubscriberPluginCanBeLoaded) {
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
      "image_transport", "image_transport::SubscriberPlugin");
  std::shared_ptr<image_transport::SubscriberPlugin> sub =
      sub_loader.createSharedInstance("image_transport/raw_sub");
  EXPECT_THAT(sub->getTransportName(), Eq("raw"));
}
