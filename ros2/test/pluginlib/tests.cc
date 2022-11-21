// Copyright 2022 Milan Vukov
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

// #include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

#include "ros2/test/pluginlib/regular_polygon.h"

// TEST(TestPluginlibClassLoader, Foo) {
int main(int, char**) {
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
      "polygon_base", "polygon_base::RegularPolygon");

  std::shared_ptr<polygon_base::RegularPolygon> triangle =
      poly_loader.createSharedInstance("polygon_plugins::Triangle");
  triangle->initialize(10.0);

  std::shared_ptr<polygon_base::RegularPolygon> square =
      poly_loader.createSharedInstance("polygon_plugins::Square");
  square->initialize(10.0);

  // printf("Triangle area: %.2f\n", triangle->area());
  // printf("Square area: %.2f\n", square->area());
}
