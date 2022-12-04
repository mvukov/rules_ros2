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

#ifndef ROS2_TEST_PLUGINLIB_REGULAR_POLYGON_H_
#define ROS2_TEST_PLUGINLIB_REGULAR_POLYGON_H_

namespace polygon_base {

class RegularPolygon {
 public:
  virtual ~RegularPolygon() = default;
  virtual void initialize(double side_length) = 0;
  virtual double area() = 0;
};

}  // namespace polygon_base

#endif  // ROS2_TEST_PLUGINLIB_REGULAR_POLYGON_H_
