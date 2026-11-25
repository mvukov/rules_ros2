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
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace {

sensor_msgs::msg::PointCloud2 MakeXyzCloud(const std::string& frame_id,
                                           int num_points) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(num_points);
  return cloud;
}

TEST(TestTf2SensorMsgs, WhenPointCloud2Transformed_EnsureFrameIdUpdated) {
  sensor_msgs::msg::PointCloud2 cloud_in = MakeXyzCloud("sensor", 3);
  sensor_msgs::msg::PointCloud2 cloud_out;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "sensor";
  transform.transform.rotation.w = 1.0;

  tf2::doTransform(cloud_in, cloud_out, transform);

  EXPECT_EQ(cloud_out.header.frame_id, "world");
}

TEST(TestTf2SensorMsgs,
     WhenIdentityTransformApplied_EnsurePointCountPreserved) {
  sensor_msgs::msg::PointCloud2 cloud_in = MakeXyzCloud("sensor", 5);
  sensor_msgs::msg::PointCloud2 cloud_out;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "sensor";
  transform.transform.rotation.w = 1.0;

  tf2::doTransform(cloud_in, cloud_out, transform);

  EXPECT_EQ(cloud_out.width, cloud_in.width);
  EXPECT_EQ(cloud_out.height, cloud_in.height);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
