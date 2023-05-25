// Copyright 2022 Open Robotics
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

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

#include "ros2/test/pluginlib/regular_polygon.h"

namespace polygon_plugins {

class Triangle : public polygon_base::RegularPolygon {
 public:
  void initialize(double side_length) override { side_length_ = side_length; }

  double area() override { return 0.5 * side_length_ * GetHeight(); }

  double GetHeight() {
    return sqrt((side_length_ * side_length_) -
                ((side_length_ / 2) * (side_length_ / 2)));
  }

 protected:
  double side_length_;
};

}  // namespace polygon_plugins

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
