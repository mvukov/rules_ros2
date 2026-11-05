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

// Checks that ros2_cpp_test runs the test via run_test_isolated, i.e. that
// the parent process is run_test_isolated and holds the reservation of the
// ROS_DOMAIN_ID of the test.

#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros2/domain_coordinator/domain_coordinator.h"

namespace {

using ::testing::Eq;
using ::testing::Ge;
using ::testing::Optional;

using domain_coordinator::DomainIdReservation;
using domain_coordinator::kPortBase;

// Returns the path of the executable of the parent process.
std::string ParentExecutable() {
  const std::string link = "/proc/" + std::to_string(getppid()) + "/exe";
  char path[PATH_MAX];
  const ssize_t length = readlink(link.c_str(), path, sizeof(path));
  return length > 0 ? std::string(path, static_cast<size_t>(length)) : "";
}

bool IsParentResolver() {
  const std::string suffix = "/run_test_isolated";
  const std::string parent = ParentExecutable();
  return parent.size() >= suffix.size() &&
         parent.compare(parent.size() - suffix.size(), suffix.size(), suffix) ==
             0;
}

// Returns the value of `variable` parsed as a domain ID, or std::nullopt if it
// is unset or not a number.
std::optional<int> ParseDomainId(const char* variable) {
  const char* value = std::getenv(variable);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  errno = 0;
  char* end = nullptr;
  const int64_t domain_id = std::strtoll(value, &end, 10);
  if (errno != 0 || *end != '\0' || domain_id < 0 || domain_id > INT_MAX) {
    return std::nullopt;
  }
  return static_cast<int>(domain_id);
}

TEST(Ros2CppTestDomainIdTests,
     GivenRos2CppTest_WhenRunning_ExpectRosDomainIdHeldByParent) {
  if (!IsParentResolver()) {
    // The resolver execs the test instead of being its parent if isolation is
    // disabled or ROS_DOMAIN_ID was given, e.g. via --test_env.
    if (std::getenv("DISABLE_ROS_ISOLATION") != nullptr ||
        std::getenv("ROS_DOMAIN_ID") != nullptr) {
      GTEST_SKIP() << "ROS domain isolation is disabled by the environment";
    }
    FAIL() << "Not run via run_test_isolated: " << ParentExecutable();
  }
  const std::optional<int> domain_id = ParseDomainId("ROS_DOMAIN_ID");
  ASSERT_THAT(domain_id, Optional(Ge(0)));

  EXPECT_THAT(DomainIdReservation::Reserve(kPortBase, *domain_id, *domain_id),
              Eq(std::nullopt));
}

}  // namespace
