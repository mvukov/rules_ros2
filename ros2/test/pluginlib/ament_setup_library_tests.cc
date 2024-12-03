// Copyright 2023 Wouter van der Stoel
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
#include "ros2/test/pluginlib/square_ament_setup/ament_setup.h"
#include "ros2/test/pluginlib/triangle_ament_setup/ament_setup.h"

using ::testing::DoubleNear;
using ::testing::Eq;

class TestAmentSetup : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
  }

  void TearDown() override { unsetenv("AMENT_PREFIX_PATH"); }
};

TEST_F(TestAmentSetup, PluginLoadingDoesNotWorkUntilAmentPrefixPathIsSet) {
  EXPECT_ANY_THROW(
      pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
          "polygon_base", "polygon_base::RegularPolygon"));
}

TEST_F(TestAmentSetup, SetupSingleAmentPrefixPath) {
  ::triangle_ament_setup::SetUpAmentPrefixPath();
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
      "polygon_base", "polygon_base::RegularPolygon");

  std::shared_ptr<polygon_base::RegularPolygon> triangle =
      poly_loader.createSharedInstance("polygon_plugins::Triangle");
  triangle->initialize(10.0);

  EXPECT_THAT(triangle->area(), DoubleNear(43.3013, 1e-4));

  EXPECT_ANY_THROW(poly_loader.createSharedInstance("polygon_plugins::Square"));
}

TEST_F(TestAmentSetup, AppendAmentPrefixPath) {
  ::triangle_ament_setup::SetUpAmentPrefixPath(/* allow_append = */ true);
  ::square_ament_setup::SetUpAmentPrefixPath(
      /* allow_append = */ true);

  {
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base", "polygon_base::RegularPolygon");

    std::shared_ptr<polygon_base::RegularPolygon> triangle =
        poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    EXPECT_THAT(triangle->area(), DoubleNear(43.3013, 1e-4));
  }

  // This will reset the AMENT_PREFIX_PATH to only contain the triangle library.
  ::triangle_ament_setup::SetUpAmentPrefixPath();
  {
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base", "polygon_base::RegularPolygon");
    EXPECT_ANY_THROW(
        poly_loader.createSharedInstance("polygon_plugins::Square"));
  }

  // Only now we can use the square library.
  ::square_ament_setup::SetUpAmentPrefixPath();
  {
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base", "polygon_base::RegularPolygon");
    EXPECT_NO_THROW(
        poly_loader.createSharedInstance("polygon_plugins::Square"));
  }
}
