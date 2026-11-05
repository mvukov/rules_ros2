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

// A C++ port of domain_coordinator from
// https://github.com/ros2/ament_cmake_ros/tree/jazzy.
//
// To coordinate ROS_DOMAIN_IDs between multiple processes, a TCP port
// kPortBase + ROS_DOMAIN_ID is bound. If binding succeeds, the ROS_DOMAIN_ID
// is reserved for as long as the socket stays open. Note that every descendant
// that doesn't close inherited file descriptors (e.g. a background job) holds
// the reservation as well, possibly beyond the lifetime of the reserving
// process.

#ifndef ROS2_ROS_DOMAIN_ID_ROS_DOMAIN_ID_H_
#define ROS2_ROS_DOMAIN_ID_ROS_DOMAIN_ID_H_

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <optional>

namespace ros_domain_id {

// Matches the port base of upstream domain_coordinator, such that reservations
// are coordinated with upstream processes. It's above the DDS ports for domain
// IDs up to kMaxDomainId (7400 + 250 * domain ID + offsets). Note that it is in
// Linux's ephemeral port range: if a port is taken by an outgoing connection,
// the corresponding domain ID is skipped.
inline constexpr int kPortBase = 32768;
inline constexpr int kMinDomainId = 1;
inline constexpr int kMaxDomainId = 100;

// Returns the first domain ID in [`min_domain_id`, `max_domain_id`] for which
// `try_bind(domain_id)` returns 0. A return value of EADDRINUSE means that the
// domain ID is taken and the next one is tried; any other value is treated as
// an errno that makes further attempts pointless. Returns std::nullopt if no
// domain ID could be bound. Errors are printed to stderr.
template <typename TryBind>
std::optional<int> FindFreeDomainId(int min_domain_id, int max_domain_id,
                                    TryBind&& try_bind) {
  for (int domain_id = min_domain_id; domain_id <= max_domain_id; ++domain_id) {
    const int bind_errno = try_bind(domain_id);
    if (bind_errno == 0) {
      return domain_id;
    }
    if (bind_errno != EADDRINUSE) {
      std::fprintf(stderr, "Failed to reserve ROS domain ID %d: %s\n",
                   domain_id, std::strerror(bind_errno));
      return std::nullopt;
    }
  }
  std::fprintf(stderr,
               "Failed to reserve a ROS domain ID in [%d, %d]: all are taken\n",
               min_domain_id, max_domain_id);
  return std::nullopt;
}

enum class Mode {
  // Run the command as is.
  kPassThrough,
  // Run the command with a reserved ROS_DOMAIN_ID.
  kIsolate,
};

// Decides the mode from the values of the environment variables ROS_DOMAIN_ID
// and DISABLE_ROS_ISOLATION, where nullptr means unset. A non-empty
// ROS_DOMAIN_ID is respected. DISABLE_ROS_ISOLATION disables isolation if set
// to any value, including an empty one or "0".
Mode DecideMode(const char* ros_domain_id, const char* disable_ros_isolation);

// Holds a reserved domain ID; the reservation is released on destruction.
class DomainIdReservation {
 public:
  // Returns a reservation of the first domain ID in [`min_domain_id`,
  // `max_domain_id`] whose port `port_base` + domain ID could be bound, or
  // std::nullopt if none could. Requires 0 <= `min_domain_id` <=
  // `max_domain_id` and all ports to be in [1, 65535]; otherwise returns
  // std::nullopt. Errors are printed to stderr.
  [[nodiscard]] static std::optional<DomainIdReservation> Reserve(
      int port_base, int min_domain_id, int max_domain_id);

  DomainIdReservation(DomainIdReservation&& other) noexcept;
  // Not needed by std::optional, which only move-constructs; deleting it keeps
  // the socket ownership simple.
  DomainIdReservation& operator=(DomainIdReservation&&) = delete;
  ~DomainIdReservation();

  int domain_id() const { return domain_id_; }

 private:
  DomainIdReservation(int domain_id, int socket_fd);

  int domain_id_;
  int socket_fd_;
};

}  // namespace ros_domain_id

#endif  // ROS2_ROS_DOMAIN_ID_ROS_DOMAIN_ID_H_
