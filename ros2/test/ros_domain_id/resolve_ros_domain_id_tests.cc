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

// Checks that ros2_cpp_test runs the test via resolve_ros_domain_id, i.e. that
// the ROS_DOMAIN_ID of the test is reserved while the test runs.

#include <cstdlib>
#include <optional>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros2/ros_domain_id/ros_domain_id.h"

namespace {

using ::testing::Eq;
using ::testing::NotNull;

using ros_domain_id::DomainIdReservation;
using ros_domain_id::kPortBase;

TEST(ResolveRosDomainIdTests,
     GivenRosDomainIdOfTest_WhenReservingSameDomainId_ExpectNoReservation) {
  const char* ros_domain_id = std::getenv("ROS_DOMAIN_ID");
  ASSERT_THAT(ros_domain_id, NotNull());
  const int domain_id = std::atoi(ros_domain_id);

  EXPECT_THAT(DomainIdReservation::Reserve(kPortBase, domain_id, domain_id),
              Eq(std::nullopt));
}

}  // namespace
