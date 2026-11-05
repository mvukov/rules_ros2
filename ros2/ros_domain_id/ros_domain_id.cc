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

#include "ros2/ros_domain_id/ros_domain_id.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ros_domain_id {
namespace {

// Closes `socket_fd` and prints an error on failure.
void CloseSocket(int socket_fd) {
  if (close(socket_fd) != 0) {
    std::fprintf(stderr, "Failed to close socket %d: %s\n", socket_fd,
                 std::strerror(errno));
  }
}

}  // namespace

std::optional<DomainIdReservation> DomainIdReservation::Reserve(
    int port_base, int min_domain_id, int max_domain_id) {
  constexpr int kMaxPort = 65535;
  if (min_domain_id < 0 || min_domain_id > max_domain_id ||
      port_base + min_domain_id < 1 || port_base + max_domain_id > kMaxPort) {
    std::fprintf(stderr, "Invalid domain ID range [%d, %d] for port base %d\n",
                 min_domain_id, max_domain_id, port_base);
    return std::nullopt;
  }
  // Not close-on-exec: an exec'd process inherits the socket and thereby
  // holds the reservation.
  const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0) {
    std::fprintf(stderr, "Failed to create a socket: %s\n",
                 std::strerror(errno));
    return std::nullopt;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  // On Linux, a failed bind leaves the socket unbound, so binding is retried
  // on the same socket for each candidate domain ID.
  int bind_errno = 0;
  for (int domain_id = min_domain_id; domain_id <= max_domain_id; ++domain_id) {
    address.sin_port = htons(static_cast<uint16_t>(port_base + domain_id));
    if (bind(socket_fd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) == 0) {
      return DomainIdReservation(domain_id, socket_fd);
    }
    bind_errno = errno;
  }
  std::fprintf(
      stderr, "Failed to reserve a ROS domain ID in [%d, %d]. Last error: %s\n",
      min_domain_id, max_domain_id, std::strerror(bind_errno));
  CloseSocket(socket_fd);
  return std::nullopt;
}

DomainIdReservation::DomainIdReservation(int domain_id, int socket_fd)
    : domain_id_(domain_id), socket_fd_(socket_fd) {}

DomainIdReservation::DomainIdReservation(DomainIdReservation&& other) noexcept
    : domain_id_(other.domain_id_), socket_fd_(other.socket_fd_) {
  other.socket_fd_ = -1;
}

DomainIdReservation::~DomainIdReservation() {
  if (socket_fd_ >= 0) {
    CloseSocket(socket_fd_);
  }
}

}  // namespace ros_domain_id
