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
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

namespace {

geometry_msgs::msg::TransformStamped MakeIdentityTransform(
    const std::string& parent_frame, const std::string& child_frame) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;
  t.transform.rotation.w = 1.0;
  return t;
}

TEST(TestTf2BufferCore, WhenNoTransformAvailable_EnsureLookupThrows) {
  tf2::BufferCore buffer;
  EXPECT_THROW(buffer.lookupTransform("target", "source", tf2::TimePointZero),
               tf2::LookupException);
}

TEST(TestTf2BufferCore, WhenTransformSet_EnsureLookupSucceeds) {
  tf2::BufferCore buffer;
  buffer.setTransform(MakeIdentityTransform("base", "child"), "test",
                      /*is_static=*/false);

  const auto result =
      buffer.lookupTransform("base", "child", tf2::TimePointZero);
  EXPECT_EQ(result.header.frame_id, "base");
  EXPECT_EQ(result.child_frame_id, "child");
  EXPECT_DOUBLE_EQ(result.transform.rotation.w, 1.0);
}

TEST(TestTf2BufferCore, WhenTransformSet_EnsureCanTransformReturnsTrue) {
  tf2::BufferCore buffer;
  buffer.setTransform(MakeIdentityTransform("base", "child"), "test",
                      /*is_static=*/false);

  EXPECT_TRUE(buffer.canTransform("base", "child", tf2::TimePointZero));
}

TEST(TestTf2BufferCore, WhenFrameAdded_EnsureItAppearsInFrameList) {
  tf2::BufferCore buffer;
  buffer.setTransform(MakeIdentityTransform("base", "child"), "test",
                      /*is_static=*/false);

  const std::vector<std::string> frames = buffer.getAllFrameNames();
  EXPECT_NE(std::find(frames.begin(), frames.end(), "child"), frames.end());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
