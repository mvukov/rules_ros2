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

#include <memory>

#include "console_bridge/console.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"
#include "rcutils/logging.h"

#include "ros2/test/pluginlib/regular_polygon.h"

using ::testing::DoubleNear;
using ::testing::Eq;

TEST(TestPluginlibClassLoader,
     WhenPluginsAvailable_EnsurePluginsCanBeLoadedAndWork) {
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
      "polygon_base", "polygon_base::RegularPolygon");

  std::shared_ptr<polygon_base::RegularPolygon> triangle =
      poly_loader.createSharedInstance("polygon_plugins::Triangle");
  triangle->initialize(10.0);

  std::shared_ptr<polygon_base::RegularPolygon> square =
      poly_loader.createSharedInstance("polygon_plugins::Square");
  square->initialize(10.0);

  EXPECT_THAT(triangle->area(), DoubleNear(43.3013, 1e-4));
  EXPECT_THAT(square->area(), Eq(100.0));
}
