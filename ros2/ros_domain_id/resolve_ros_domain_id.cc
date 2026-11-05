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

// Usage: resolve_ros_domain_id <command> [<args>...]
//
// A C++ port of run_test_isolated.py from
// https://github.com/ros2/ament_cmake_ros/tree/jazzy.
//
// Runs the given command with a ROS_DOMAIN_ID that is not used by any other
// resolve_ros_domain_id process, such that commands (e.g. tests) can run in
// parallel without ROS cross-talk. If ROS_DOMAIN_ID is already set to a
// non-empty value, it is respected. Isolation can be disabled by setting
// DISABLE_ROS_ISOLATION to any value, including an empty one or "0".
//
// The reservation is held by the command and by all of its descendants that
// don't close inherited file descriptors.

#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>

#include "ros2/ros_domain_id/ros_domain_id.h"

namespace {

// Exit codes following the shell convention.
constexpr int kCommandNotFoundExitCode = 127;
constexpr int kCommandNotExecutableExitCode = 126;

// Replaces the current process with `argv[0]` searched in PATH.
// Returns only on failure.
int Exec(char** argv) {
  execvp(argv[0], argv);
  const int exec_errno = errno;
  std::fprintf(stderr, "Failed to execute '%s': %s\n", argv[0],
               std::strerror(exec_errno));
  return exec_errno == ENOENT ? kCommandNotFoundExitCode
                              : kCommandNotExecutableExitCode;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "There should be at least one argument given to "
                 "resolve_ros_domain_id\n");
    return EXIT_FAILURE;
  }
  const ros_domain_id::Mode mode = ros_domain_id::DecideMode(
      std::getenv("ROS_DOMAIN_ID"), std::getenv("DISABLE_ROS_ISOLATION"));
  if (mode == ros_domain_id::Mode::kPassThrough) {
    return Exec(argv + 1);
  }

  // The reservation socket is inherited by the command, which then holds the
  // reservation for its whole lifetime.
  const std::optional<ros_domain_id::DomainIdReservation> reservation =
      ros_domain_id::DomainIdReservation::Reserve(ros_domain_id::kPortBase,
                                                  ros_domain_id::kMinDomainId,
                                                  ros_domain_id::kMaxDomainId);
  if (!reservation.has_value()) {
    return EXIT_FAILURE;
  }
  const std::string domain_id = std::to_string(reservation->domain_id());
  if (setenv("ROS_DOMAIN_ID", domain_id.c_str(), /*overwrite=*/1) != 0) {
    std::fprintf(stderr, "Failed to set ROS_DOMAIN_ID: %s\n",
                 std::strerror(errno));
    return EXIT_FAILURE;
  }
  return Exec(argv + 1);
}
