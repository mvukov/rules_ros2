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

// Usage: probe report | probe exit <code> | probe kill
//
// A child command for resolve_ros_domain_id_tests:
// - report: prints "ros_domain_id=<value or <unset>> held=<0|1> extra_fds=<n>"
//   where held tells whether the port of ROS_DOMAIN_ID is in use and
//   extra_fds is the number of open fds other than stdin/stdout/stderr.
// - exit <code>: exits with <code>.
// - kill: kills itself with SIGKILL.

#include <dirent.h>
#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>

#include "ros2/ros_domain_id/ros_domain_id.h"

namespace {

int CountExtraFds() {
  DIR* dir = opendir("/proc/self/fd");
  if (dir == nullptr) {
    return -1;
  }
  int count = 0;
  while (const dirent* entry = readdir(dir)) {
    if (entry->d_name[0] == '.') {
      continue;
    }
    const int fd = std::atoi(entry->d_name);
    if (fd > STDERR_FILENO && fd != dirfd(dir)) {
      ++count;
    }
  }
  closedir(dir);
  return count;
}

int Report() {
  const char* ros_domain_id = std::getenv("ROS_DOMAIN_ID");
  bool held = false;
  if (ros_domain_id != nullptr) {
    const int domain_id = std::atoi(ros_domain_id);
    held = !ros_domain_id::DomainIdReservation::Reserve(
                ros_domain_id::kPortBase, domain_id, domain_id)
                .has_value();
  }
  std::printf("ros_domain_id=%s held=%d extra_fds=%d\n",
              ros_domain_id != nullptr ? ros_domain_id : "<unset>",
              held ? 1 : 0, CountExtraFds());
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::strcmp(argv[1], "report") == 0) {
    return Report();
  }
  if (argc == 3 && std::strcmp(argv[1], "exit") == 0) {
    return std::atoi(argv[2]);
  }
  if (argc == 2 && std::strcmp(argv[1], "kill") == 0) {
    raise(SIGKILL);
  }
  std::fprintf(stderr, "Invalid arguments\n");
  return EXIT_FAILURE;
}
