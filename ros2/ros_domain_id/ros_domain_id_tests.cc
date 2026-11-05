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

#include <optional>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace ros_domain_id {
namespace {

using ::testing::Eq;
using ::testing::Optional;
using ::testing::Property;
using ::testing::SizeIs;

// Above Linux's ephemeral port range (32768-60999) and the DDS ports, such
// that the tested ports are neither taken by outgoing connections nor by
// resolve_ros_domain_id processes running at the same time.
constexpr int kTestPortBase = 61000;

std::optional<DomainIdReservation> ReserveForTest() {
  return DomainIdReservation::Reserve(kTestPortBase, kMinDomainId,
                                      kMaxDomainId);
}

// Matches a std::optional<DomainIdReservation> that holds `domain_id`.
auto HoldsDomainId(int domain_id) {
  return Optional(Property(&DomainIdReservation::domain_id, Eq(domain_id)));
}

// Reserves all domain IDs, expecting them to be reserved in increasing order.
std::vector<DomainIdReservation> ReserveAll() {
  std::vector<DomainIdReservation> reservations;
  for (int domain_id = kMinDomainId; domain_id <= kMaxDomainId; ++domain_id) {
    std::optional<DomainIdReservation> reservation = ReserveForTest();
    EXPECT_THAT(reservation, HoldsDomainId(domain_id));
    if (!reservation.has_value()) {
      break;
    }
    reservations.push_back(std::move(*reservation));
  }
  return reservations;
}

TEST(TestDomainIdReservation,
     GivenNoReservations_WhenReserving_ExpectMinDomainId) {
  EXPECT_THAT(ReserveForTest(), HoldsDomainId(kMinDomainId));
}

TEST(TestDomainIdReservation,
     GivenHeldReservation_WhenReserving_ExpectNextDomainId) {
  const std::optional<DomainIdReservation> held = ReserveForTest();
  ASSERT_THAT(held, HoldsDomainId(kMinDomainId));

  EXPECT_THAT(ReserveForTest(), HoldsDomainId(kMinDomainId + 1));
}

TEST(TestDomainIdReservation,
     GivenReleasedReservation_WhenReserving_ExpectSameDomainIdReused) {
  {
    const std::optional<DomainIdReservation> released = ReserveForTest();
    ASSERT_THAT(released, HoldsDomainId(kMinDomainId));
  }

  EXPECT_THAT(ReserveForTest(), HoldsDomainId(kMinDomainId));
}

TEST(TestDomainIdReservation,
     GivenAllDomainIdsReserved_WhenReserving_ExpectNoReservation) {
  const std::vector<DomainIdReservation> held = ReserveAll();
  ASSERT_THAT(held, SizeIs(kMaxDomainId - kMinDomainId + 1));

  EXPECT_THAT(ReserveForTest(), Eq(std::nullopt));
}

TEST(
    TestDomainIdReservation,
    GivenSingleDomainIdReserved_WhenReservingSameDomainId_ExpectNoReservation) {
  constexpr int kDomainId = 42;
  const std::optional<DomainIdReservation> held =
      DomainIdReservation::Reserve(kTestPortBase, kDomainId, kDomainId);
  ASSERT_THAT(held, HoldsDomainId(kDomainId));

  EXPECT_THAT(DomainIdReservation::Reserve(kTestPortBase, kDomainId, kDomainId),
              Eq(std::nullopt));
}

TEST(TestDomainIdReservation,
     GivenReservation_WhenMoved_ExpectReservationKept) {
  std::optional<DomainIdReservation> original = ReserveForTest();
  ASSERT_THAT(original, HoldsDomainId(kMinDomainId));

  const DomainIdReservation moved(std::move(*original));
  original.reset();

  EXPECT_THAT(moved,
              Property(&DomainIdReservation::domain_id, Eq(kMinDomainId)));
  EXPECT_THAT(ReserveForTest(), HoldsDomainId(kMinDomainId + 1));
}

TEST(TestDomainIdReservation,
     GivenInvalidRange_WhenReserving_ExpectNoReservation) {
  EXPECT_THAT(DomainIdReservation::Reserve(kTestPortBase, 2, 1),
              Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(kTestPortBase, -1, 1),
              Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(0, 0, 1), Eq(std::nullopt));
  EXPECT_THAT(DomainIdReservation::Reserve(65535, 0, 1), Eq(std::nullopt));
}

}  // namespace
}  // namespace ros_domain_id
