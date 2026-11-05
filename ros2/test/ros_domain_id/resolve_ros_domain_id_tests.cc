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

// Black-box tests of resolve_ros_domain_id, which is run with an explicitly
// built environment, such that the environment of the test doesn't matter.

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros2/ros_domain_id/ros_domain_id.h"

namespace {

using ::testing::Eq;
using ::testing::MatchesRegex;
using ::testing::NotNull;

struct Result {
  int status = -1;
  std::string output;
};

// Runs the resolver with `args` and the environment `env`, and returns its wait
// status and stdout.
Result RunResolver(const std::vector<std::string>& args,
                   const std::vector<std::string>& env) {
  const char* resolver = std::getenv("RESOLVE_ROS_DOMAIN_ID");
  EXPECT_THAT(resolver, NotNull());
  if (resolver == nullptr) {
    return {};
  }

  std::vector<std::string> argv_storage = {resolver};
  argv_storage.insert(argv_storage.end(), args.begin(), args.end());
  std::vector<char*> argv;
  for (std::string& arg : argv_storage) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);
  std::vector<std::string> env_storage = env;
  std::vector<char*> envp;
  for (std::string& variable : env_storage) {
    envp.push_back(variable.data());
  }
  envp.push_back(nullptr);

  int pipe_fds[2];
  EXPECT_THAT(pipe2(pipe_fds, O_CLOEXEC), Eq(0));
  posix_spawn_file_actions_t file_actions;
  posix_spawn_file_actions_init(&file_actions);
  posix_spawn_file_actions_adddup2(&file_actions, pipe_fds[1], STDOUT_FILENO);
  pid_t pid = 0;
  const int spawn_error = posix_spawn(&pid, resolver, &file_actions, nullptr,
                                      argv.data(), envp.data());
  posix_spawn_file_actions_destroy(&file_actions);
  close(pipe_fds[1]);
  EXPECT_THAT(spawn_error, Eq(0));

  Result result;
  char buffer[256];
  ssize_t num_read = 0;
  while ((num_read = read(pipe_fds[0], buffer, sizeof(buffer))) > 0) {
    result.output.append(buffer, static_cast<size_t>(num_read));
  }
  close(pipe_fds[0]);
  if (spawn_error == 0) {
    EXPECT_THAT(waitpid(pid, &result.status, 0), Eq(pid));
  }
  return result;
}

std::string Probe() {
  const char* probe = std::getenv("PROBE");
  EXPECT_THAT(probe, NotNull());
  return probe != nullptr ? probe : "";
}

int ExitCode(const Result& result) {
  return WIFEXITED(result.status) ? WEXITSTATUS(result.status) : -1;
}

TEST(ResolveRosDomainIdTests,
     GivenNoRosDomainId_WhenRunning_ExpectReservedDomainIdAndNoExtraFds) {
  const Result result = RunResolver({Probe(), "report"}, {});

  EXPECT_THAT(ExitCode(result), Eq(0));
  EXPECT_THAT(result.output,
              MatchesRegex("ros_domain_id=[0-9]+ held=1 extra_fds=0\n"));
}

TEST(ResolveRosDomainIdTests,
     GivenEmptyRosDomainId_WhenRunning_ExpectReservedDomainId) {
  const Result result = RunResolver({Probe(), "report"}, {"ROS_DOMAIN_ID="});

  EXPECT_THAT(result.output,
              MatchesRegex("ros_domain_id=[0-9]+ held=1 extra_fds=0\n"));
}

TEST(ResolveRosDomainIdTests,
     GivenRosDomainId_WhenRunning_ExpectRosDomainIdRespected) {
  const Result result = RunResolver({Probe(), "report"}, {"ROS_DOMAIN_ID=7"});

  EXPECT_THAT(ExitCode(result), Eq(0));
  EXPECT_THAT(result.output, MatchesRegex("ros_domain_id=7 held=. .*\n"));
}

TEST(ResolveRosDomainIdTests,
     GivenDisableRosIsolation_WhenRunning_ExpectNoRosDomainId) {
  const Result result =
      RunResolver({Probe(), "report"}, {"DISABLE_ROS_ISOLATION="});

  EXPECT_THAT(result.output, MatchesRegex("ros_domain_id=<unset> .*\n"));
}

TEST(ResolveRosDomainIdTests,
     GivenCommandExitCode_WhenRunning_ExpectSameExitCode) {
  EXPECT_THAT(ExitCode(RunResolver({Probe(), "exit", "3"}, {})), Eq(3));
  EXPECT_THAT(ExitCode(RunResolver({Probe(), "exit", "0"}, {})), Eq(0));
}

TEST(ResolveRosDomainIdTests,
     GivenCommandKilledBySignal_WhenRunning_ExpectKilledBySameSignal) {
  const Result result = RunResolver({Probe(), "kill"}, {});

  ASSERT_TRUE(WIFSIGNALED(result.status));
  EXPECT_THAT(WTERMSIG(result.status), Eq(SIGKILL));
}

TEST(ResolveRosDomainIdTests,
     GivenMissingCommand_WhenRunning_ExpectCommandNotFoundExitCode) {
  EXPECT_THAT(ExitCode(RunResolver({"rules_ros2_nonexistent_command"},
                                   {"PATH=/nonexistent"})),
              Eq(127));
  EXPECT_THAT(ExitCode(RunResolver({"rules_ros2_nonexistent_command"},
                                   {"PATH=/nonexistent", "ROS_DOMAIN_ID=7"})),
              Eq(127));
}

TEST(ResolveRosDomainIdTests, GivenNoCommand_WhenRunning_ExpectFailure) {
  EXPECT_THAT(ExitCode(RunResolver({}, {})), Eq(EXIT_FAILURE));
}

}  // namespace
