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
// DISABLE_ROS_ISOLATION to any value, including an empty one or "0". In both
// cases, this process is replaced by the command.
//
// Otherwise, the command runs in a child process while this process holds the
// reservation, which is released once the command exits. The command doesn't
// inherit the reservation socket. SIGHUP, SIGINT and SIGTERM are forwarded to
// the command. The exit code of the command is returned; if the command is
// killed by a signal, this process kills itself with the same signal.

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
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
constexpr int kKilledBySignalExitCodeBase = 128;

constexpr int kForwardedSignals[] = {SIGHUP, SIGINT, SIGTERM};

volatile sig_atomic_t g_child_pid = 0;

void ForwardSignal(int signal_number) {
  if (g_child_pid > 0) {
    kill(g_child_pid, signal_number);
  }
}

// Replaces the current process with `argv[0]` searched in PATH.
// Returns only on failure, with the exit code to report.
int Exec(char** argv) {
  execvp(argv[0], argv);
  const int exec_errno = errno;
  std::fprintf(stderr, "Failed to execute '%s': %s\n", argv[0],
               std::strerror(exec_errno));
  return exec_errno == ENOENT ? kCommandNotFoundExitCode
                              : kCommandNotExecutableExitCode;
}

// Runs `argv[0]` searched in PATH in a child process, forwards signals to it,
// and returns its exit code once it exits. If the child is killed by a signal,
// kills the current process with the same signal.
int SpawnAndWait(char** argv) {
  // Signals are blocked until the handlers know the child's PID, such that
  // none is lost.
  sigset_t forwarded_signals;
  sigemptyset(&forwarded_signals);
  for (const int signal_number : kForwardedSignals) {
    sigaddset(&forwarded_signals, signal_number);
  }
  sigset_t original_mask;
  sigprocmask(SIG_BLOCK, &forwarded_signals, &original_mask);

  struct sigaction action{};
  action.sa_handler = ForwardSignal;
  action.sa_flags = SA_RESTART;
  sigemptyset(&action.sa_mask);
  for (const int signal_number : kForwardedSignals) {
    sigaction(signal_number, &action, nullptr);
  }

  const pid_t child_pid = fork();
  if (child_pid < 0) {
    std::fprintf(stderr, "Failed to fork: %s\n", std::strerror(errno));
    return EXIT_FAILURE;
  }
  if (child_pid == 0) {
    // exec resets the handlers, but not the mask.
    sigprocmask(SIG_SETMASK, &original_mask, nullptr);
    _exit(Exec(argv));
  }
  g_child_pid = child_pid;
  sigprocmask(SIG_SETMASK, &original_mask, nullptr);

  int status = 0;
  while (waitpid(child_pid, &status, 0) < 0) {
    if (errno != EINTR) {
      std::fprintf(stderr, "Failed to wait for '%s': %s\n", argv[0],
                   std::strerror(errno));
      return EXIT_FAILURE;
    }
  }
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  if (WIFSIGNALED(status)) {
    const int signal_number = WTERMSIG(status);
    signal(signal_number, SIG_DFL);
    raise(signal_number);
    return kKilledBySignalExitCodeBase + signal_number;
  }
  return EXIT_FAILURE;
}

int Run(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "There should be at least one argument given to "
                 "resolve_ros_domain_id\n");
    return EXIT_FAILURE;
  }
  char** command = argv + 1;
  const ros_domain_id::Mode mode = ros_domain_id::DecideMode(
      std::getenv("ROS_DOMAIN_ID"), std::getenv("DISABLE_ROS_ISOLATION"));
  if (mode == ros_domain_id::Mode::kPassThrough) {
    return Exec(command);
  }

  // Released on return, i.e. once the command exited.
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
  return SpawnAndWait(command);
}

}  // namespace

int main(int argc, char** argv) { return Run(argc, argv); }
