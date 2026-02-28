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

#include "point_proto_ros_msgs/proto_converters.h"
#include "transform_proto_ros_msgs/proto_converters.h"

namespace {

// ---------------------------------------------------------------------------
// Point converter tests
// ---------------------------------------------------------------------------

TEST(PointConverterTest, ToRos) {
  ros2::test::protobuf::Point proto;
  proto.set_x(1.0);
  proto.set_y(2.0);
  proto.set_z(3.0);
  proto.set_label("hello");
  proto.set_id(42);
  proto.set_valid(true);
  proto.add_values(1.5f);
  proto.add_values(2.5f);

  const auto ros = point_proto_ros_msgs::proto_converters::ToRos(proto);

  EXPECT_DOUBLE_EQ(ros.x, 1.0);
  EXPECT_DOUBLE_EQ(ros.y, 2.0);
  EXPECT_DOUBLE_EQ(ros.z, 3.0);
  EXPECT_EQ(ros.label, "hello");
  EXPECT_EQ(ros.id, 42);
  EXPECT_EQ(ros.valid, true);
  ASSERT_EQ(ros.values.size(), 2u);
  EXPECT_FLOAT_EQ(ros.values[0], 1.5f);
  EXPECT_FLOAT_EQ(ros.values[1], 2.5f);
}

TEST(PointConverterTest, FromRos) {
  point_proto_ros_msgs::msg::Point ros;
  ros.x = 4.0;
  ros.y = 5.0;
  ros.z = 6.0;
  ros.label = "world";
  ros.id = -7;
  ros.valid = false;
  ros.values = {3.0f, 4.0f, 5.0f};

  const auto proto = point_proto_ros_msgs::proto_converters::FromRos(ros);

  EXPECT_DOUBLE_EQ(proto.x(), 4.0);
  EXPECT_DOUBLE_EQ(proto.y(), 5.0);
  EXPECT_DOUBLE_EQ(proto.z(), 6.0);
  EXPECT_EQ(proto.label(), "world");
  EXPECT_EQ(proto.id(), -7);
  EXPECT_EQ(proto.valid(), false);
  ASSERT_EQ(proto.values_size(), 3);
  EXPECT_FLOAT_EQ(proto.values(0), 3.0f);
  EXPECT_FLOAT_EQ(proto.values(1), 4.0f);
  EXPECT_FLOAT_EQ(proto.values(2), 5.0f);
}

TEST(PointConverterTest, RoundTrip) {
  ros2::test::protobuf::Point original;
  original.set_x(10.0);
  original.set_y(20.0);
  original.set_z(30.0);
  original.set_label("round");
  original.set_id(99);
  original.set_valid(true);
  original.add_values(0.1f);
  original.add_values(0.2f);

  const auto ros = point_proto_ros_msgs::proto_converters::ToRos(original);
  const auto recovered = point_proto_ros_msgs::proto_converters::FromRos(ros);

  EXPECT_EQ(original.SerializeAsString(), recovered.SerializeAsString());
}

// ---------------------------------------------------------------------------
// Transform converter tests
// ---------------------------------------------------------------------------

TEST(TransformConverterTest, ToRos) {
  ros2::test::protobuf::Transform proto;
  proto.mutable_point()->set_x(7.0);
  proto.mutable_point()->set_y(8.0);
  proto.mutable_point()->set_z(9.0);
  proto.mutable_point()->set_label("pt");
  proto.mutable_point()->set_id(1);
  proto.mutable_point()->set_valid(true);

  const auto ros = transform_proto_ros_msgs::proto_converters::ToRos(proto);

  EXPECT_DOUBLE_EQ(ros.point.x, 7.0);
  EXPECT_DOUBLE_EQ(ros.point.y, 8.0);
  EXPECT_DOUBLE_EQ(ros.point.z, 9.0);
  EXPECT_EQ(ros.point.label, "pt");
  EXPECT_EQ(ros.point.id, 1);
  EXPECT_EQ(ros.point.valid, true);
}

TEST(TransformConverterTest, RoundTrip) {
  ros2::test::protobuf::Transform original;
  original.mutable_point()->set_x(1.1);
  original.mutable_point()->set_y(2.2);
  original.mutable_point()->set_z(3.3);
  original.mutable_point()->set_label("trip");
  original.mutable_point()->set_id(5);
  original.mutable_point()->set_valid(false);
  original.mutable_point()->add_values(0.5f);

  const auto ros = transform_proto_ros_msgs::proto_converters::ToRos(original);
  const auto recovered =
      transform_proto_ros_msgs::proto_converters::FromRos(ros);

  EXPECT_EQ(original.SerializeAsString(), recovered.SerializeAsString());
}

}  // namespace
