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

#include "ros2/domain_coordinator/domain_coordinator.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <optional>
#include <set>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace domain_coordinator {
namespace {

using ::testing::Eq;
using ::testing::Ge;
using ::testing::Optional;
using ::testing::Property;

// Returns a binder for FindFreeDomainId for which `taken` domain IDs are in use
// and counts the calls in `num_calls`.
auto FakeBinder(std::set<int> taken, int* num_calls) {
  return [taken = std::move(taken), num_calls](int domain_id) {
    ++*num_calls;
    return taken.count(domain_id) > 0 ? EADDRINUSE : 0;
  };
}

TEST(TestFindFreeDomainId, GivenAllFree_WhenFinding_ExpectMinDomainId) {
  int num_calls = 0;
  EXPECT_THAT(FindFreeDomainId(1, 100, FakeBinder({}, &num_calls)),
              Optional(1));
  EXPECT_THAT(num_calls, Eq(1));
}

TEST(TestFindFreeDomainId, GivenLowestTaken_WhenFinding_ExpectNextFree) {
  int num_calls = 0;
  EXPECT_THAT(FindFreeDomainId(1, 100, FakeBinder({1, 2}, &num_calls)),
              Optional(3));
  EXPECT_THAT(num_calls, Eq(3));
}

TEST(TestFindFreeDomainId, GivenAllTaken_WhenFinding_ExpectNothing) {
  int num_calls = 0;
  EXPECT_THAT(FindFreeDomainId(1, 3, FakeBinder({1, 2, 3}, &num_calls)),
              Eq(std::nullopt));
  EXPECT_THAT(num_calls, Eq(3));
}

TEST(TestFindFreeDomainId, GivenSingleFreeDomainId_WhenFinding_ExpectIt) {
  int num_calls = 0;
  EXPECT_THAT(FindFreeDomainId(42, 42, FakeBinder({}, &num_calls)),
              Optional(42));
}

TEST(TestFindFreeDomainId, GivenOtherError_WhenFinding_ExpectNoRetry) {
  int num_calls = 0;
  const auto binder = [&num_calls](int) {
    ++num_calls;
    return EACCES;
  };
  EXPECT_THAT(FindFreeDomainId(1, 100, binder), Eq(std::nullopt));
  EXPECT_THAT(num_calls, Eq(1));
}

TEST(TestDecideMode, GivenNoVariables_WhenDeciding_ExpectIsolate) {
  EXPECT_THAT(DecideMode(nullptr, nullptr), Eq(Mode::kIsolate));
}

TEST(TestDecideMode, GivenEmptyRosDomainId_WhenDeciding_ExpectIsolate) {
  EXPECT_THAT(DecideMode("", nullptr), Eq(Mode::kIsolate));
}

TEST(TestDecideMode, GivenRosDomainId_WhenDeciding_ExpectPassThrough) {
  EXPECT_THAT(DecideMode("7", nullptr), Eq(Mode::kPassThrough));
}

TEST(TestDecideMode, GivenDisableRosIsolation_WhenDeciding_ExpectPassThrough) {
  EXPECT_THAT(DecideMode(nullptr, ""), Eq(Mode::kPassThrough));
  EXPECT_THAT(DecideMode(nullptr, "0"), Eq(Mode::kPassThrough));
  EXPECT_THAT(DecideMode("", "1"), Eq(Mode::kPassThrough));
  EXPECT_THAT(DecideMode("7", "1"), Eq(Mode::kPassThrough));
}

// Holds a TCP socket bound to a port chosen by the kernel.
class PortHolder {
 public:
  PortHolder() : socket_fd_(socket(AF_INET, SOCK_STREAM, 0)) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = 0;
    socklen_t length = sizeof(address);
    if (socket_fd_ >= 0 &&
        bind(socket_fd_, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) == 0 &&
        getsockname(socket_fd_, reinterpret_cast<sockaddr*>(&address),
                    &length) == 0) {
      port_ = ntohs(address.sin_port);
    }
  }
  PortHolder(const PortHolder&) = delete;
  PortHolder& operator=(const PortHolder&) = delete;
  ~PortHolder() { Release(); }

  // Returns the bound port, or -1 if binding failed.
  int port() const { return port_; }

  void Release() {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

 private:
  int socket_fd_;
  int port_ = -1;
};

// Matches a std::optional<DomainIdReservation> that holds `domain_id`.
auto HoldsDomainId(int domain_id) {
  return Optional(Property(&DomainIdReservation::domain_id, Eq(domain_id)));
}

// The tests below use ports from the kernel instead of fixed ones, such that
// they don't depend on which ports happen to be in use on the host.

TEST(TestDomainIdReservation,
     GivenPortInUse_WhenReserving_ExpectNoReservationUntilReleased) {
  PortHolder holder;
  ASSERT_THAT(holder.port(), Ge(1));

  EXPECT_THAT(DomainIdReservation::Reserve(holder.port(), 0, 0),
              Eq(std::nullopt));

  holder.Release();
  EXPECT_THAT(DomainIdReservation::Reserve(holder.port(), 0, 0),
              HoldsDomainId(0));
}

TEST(TestDomainIdReservation,
     GivenHeldReservation_WhenReservingSameDomainId_ExpectNoReservation) {
  PortHolder holder;
  ASSERT_THAT(holder.port(), Ge(1));
  holder.Release();

  const std::optional<DomainIdReservation> held =
      DomainIdReservation::Reserve(holder.port(), 0, 0);
  ASSERT_THAT(held, HoldsDomainId(0));

  EXPECT_THAT(DomainIdReservation::Reserve(holder.port(), 0, 0),
              Eq(std::nullopt));
}

TEST(TestDomainIdReservation,
     GivenReleasedReservation_WhenReserving_ExpectSameDomainIdReused) {
  PortHolder holder;
  ASSERT_THAT(holder.port(), Ge(1));
  holder.Release();

  {
    const std::optional<DomainIdReservation> released =
        DomainIdReservation::Reserve(holder.port(), 0, 0);
    ASSERT_THAT(released, HoldsDomainId(0));
  }

  EXPECT_THAT(DomainIdReservation::Reserve(holder.port(), 0, 0),
              HoldsDomainId(0));
}

TEST(TestDomainIdReservation,
     GivenReservation_WhenMoved_ExpectReservationKept) {
  PortHolder holder;
  ASSERT_THAT(holder.port(), Ge(1));
  holder.Release();

  std::optional<DomainIdReservation> original =
      DomainIdReservation::Reserve(holder.port(), 0, 0);
  ASSERT_THAT(original, HoldsDomainId(0));

  const DomainIdReservation moved(std::move(*original));
  original.reset();

  EXPECT_THAT(moved, Property(&DomainIdReservation::domain_id, Eq(0)));
  EXPECT_THAT(DomainIdReservation::Reserve(holder.port(), 0, 0),
              Eq(std::nullopt));
}

TEST(TestDomainIdReservation,
     GivenInvalidRange_WhenReserving_ExpectNoReservation) {
  EXPECT_THAT(DomainIdReservation::Reserve(kPortBase, 2, 1), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(kPortBase, -1, 1), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(0, 0, 1), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(65535, 0, 1), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(INT_MAX, 0, 1), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(1, 0, INT_MAX), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(INT_MIN, INT_MAX, INT_MAX),
              Eq(std::nullopt));
}

}  // namespace
}  // namespace domain_coordinator
