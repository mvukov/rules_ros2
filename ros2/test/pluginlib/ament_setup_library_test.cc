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

#include "console_bridge/console.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"
#include "rcutils/logging.h"

#include "ros2/test/pluginlib/regular_polygon.h"
#include "ros2/test/pluginlib/square_ament_setup.hpp"
#include "ros2/test/pluginlib/triangle_ament_setup.hpp"

using ::testing::DoubleNear;
using ::testing::Eq;

TEST(TestAmentSetup, PluginLoadingDoesNotWorkUntilAmentPrefixPathIsSet) {
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  EXPECT_ANY_THROW(
      pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
          "polygon_base", "polygon_base::RegularPolygon"));
}

TEST(TestAmentSetup, SetupSingleAmentPrefixPath) {
  ::ros2_test_pluginlib_triangle_ament_setup::setup_ament_prefix_path();
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
      "polygon_base", "polygon_base::RegularPolygon");

  std::shared_ptr<polygon_base::RegularPolygon> triangle =
      poly_loader.createSharedInstance("triangle_plugin::Triangle");
  triangle->initialize(10.0);

  EXPECT_THAT(triangle->area(), DoubleNear(43.3013, 1e-4));

  EXPECT_ANY_THROW(poly_loader.createSharedInstance("square::Square"));
}

TEST(TestAmentSetup, AppendAmentPrefixPath) {
  ::ros2_test_pluginlib_triangle_ament_setup::setup_ament_prefix_path();
  ::ros2_test_pluginlib_square_ament_setup::setup_ament_prefix_path(
      /* append = */ true);

  {
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base", "polygon_base::RegularPolygon");

    std::shared_ptr<polygon_base::RegularPolygon> triangle =
        poly_loader.createSharedInstance("triangle_plugin::Triangle");
    triangle->initialize(10.0);

    EXPECT_THAT(triangle->area(), DoubleNear(43.3013, 1e-4));

    std::shared_ptr<polygon_base::RegularPolygon> square =
        poly_loader.createSharedInstance("square_plugin::Square");
    square->initialize(10.0);

    EXPECT_THAT(square->area(), Eq(100.0));
  }

  // this will reset the AMENT_PREFIX_PATH to only contain the triangle library.
  ::ros2_test_pluginlib_triangle_ament_setup::setup_ament_prefix_path();
  {
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base", "polygon_base::RegularPolygon");
    EXPECT_ANY_THROW(poly_loader.createSharedInstance("square_plugin::Square"));
  }

  // and now only the square library
  ::ros2_test_pluginlib_square_ament_setup::setup_ament_prefix_path();
  {
    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
        "polygon_base", "polygon_base::RegularPolygon");
    EXPECT_NO_THROW(poly_loader.createSharedInstance("square_plugin::Square"));
  }
}
