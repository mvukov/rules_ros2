// Copyright 2026 Milan Vukov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include "builtin_interfaces/msg/time.hpp"
#include "google/protobuf/timestamp.pb.h"

namespace rules_ros2_protobuf_common_runtime {

void ToRos(const google::protobuf::Timestamp& proto,
           builtin_interfaces::msg::Time* ros);

void FromRos(const builtin_interfaces::msg::Time& ros,
             google::protobuf::Timestamp* proto);

}  // namespace rules_ros2_protobuf_common_runtime
