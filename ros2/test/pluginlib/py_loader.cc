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

#include "console_bridge/console.h"
#include "pluginlib/class_loader.hpp"
#include "pybind11/pybind11.h"
#include "rcutils/logging.h"

#include "ros2/test/pluginlib/regular_polygon.h"

void LoadPlugins() {
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader(
      "polygon_base", "polygon_base::RegularPolygon");
  std::shared_ptr<polygon_base::RegularPolygon> triangle =
      poly_loader.createSharedInstance("triangle_plugin::Triangle");
}

PYBIND11_MODULE(py_loader, m) { m.def("load_plugins", &LoadPlugins); }
