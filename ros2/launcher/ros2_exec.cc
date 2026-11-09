// Copyright 2026 Laurenz Altenmueller
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
//
// Sets up the ROS 2 process environment and exec()s the wrapped program.
//
// Usage: ros2_exec <ament_prefix_path|-> <program> [args...]
//
// This is the second stage of a ros2_* launcher. The first stage is a
// hermetic_launcher stub (see //ros2:launcher.bzl), which resolves all
// runfiles paths to absolute paths and exports RUNFILES_DIR before exec()ing
// this program. Everything this program does is pure environment setup; it
// has no dependency on a shell or a Python interpreter.
//
// - If <ament_prefix_path> is "-", AMENT_PREFIX_PATH is unset. Otherwise it
//   is set to <ament_prefix_path>, which is made absolute via RUNFILES_DIR if
//   needed.
// - Under `bazel test` (BAZEL_TEST is set), ROS_HOME and ROS_LOG_DIR default
//   to TEST_UNDECLARED_OUTPUTS_DIR (or TEST_TMPDIR) unless already set.

#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

const char* GetEnvOrNull(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return nullptr;
  }
  return value;
}

void SetEnvIfUnset(const char* name, const char* value) {
  if (GetEnvOrNull(name) == nullptr) {
    setenv(name, value, /*overwrite=*/1);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "Usage: %s <ament_prefix_path|-> <program> [args...]\n",
                 argv[0]);
    return 2;
  }

  if (GetEnvOrNull("BAZEL_TEST") != nullptr) {
    const char* test_output_dir = GetEnvOrNull("TEST_UNDECLARED_OUTPUTS_DIR");
    if (test_output_dir == nullptr) {
      test_output_dir = GetEnvOrNull("TEST_TMPDIR");
    }
    if (test_output_dir != nullptr) {
      SetEnvIfUnset("ROS_HOME", test_output_dir);
      SetEnvIfUnset("ROS_LOG_DIR", test_output_dir);
    }
  }

  std::string ament_prefix_path = argv[1];
  if (ament_prefix_path == "-") {
    unsetenv("AMENT_PREFIX_PATH");
  } else {
    if (ament_prefix_path[0] != '/') {
      // The launcher could not resolve the path (e.g. manifest-only runfiles).
      // Fall back to joining it with the runfiles directory.
      const char* runfiles_dir = GetEnvOrNull("RUNFILES_DIR");
      if (runfiles_dir != nullptr) {
        ament_prefix_path = std::string(runfiles_dir) + "/" + ament_prefix_path;
      }
    }
    setenv("AMENT_PREFIX_PATH", ament_prefix_path.c_str(), /*overwrite=*/1);
  }

  execv(argv[2], argv + 2);
  std::fprintf(stderr, "ros2_exec: failed to exec %s: %s\n", argv[2],
               std::strerror(errno));
  return 127;
}
