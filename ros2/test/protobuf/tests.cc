// Copyright 2026 Milan Vukov
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
#include "gtest/gtest.h"

#include "point_proto_ros_msgs/msg/dummy_one.hpp"
#include "point_proto_ros_msgs/msg/point.hpp"
#include "transform_proto_ros_msgs/msg/transform.hpp"

namespace {

TEST(DummyOneTest, EnumConstants) {
  using DummyOne = point_proto_ros_msgs::msg::DummyOne;
  EXPECT_EQ(DummyOne::COLOR_UNKNOWN, 0);
  EXPECT_EQ(DummyOne::COLOR_RED, 1);
  EXPECT_EQ(DummyOne::COLOR_GREEN, 2);
  EXPECT_EQ(DummyOne::COLOR_BLUE, 3);
}

}  // namespace
