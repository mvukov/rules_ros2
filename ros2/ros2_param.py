# Copyright 2022 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import sys

sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_NOW)

# ruff: noqa: E402
import ros2cli.cli
import ros2param.verb.delete
import ros2param.verb.describe
import ros2param.verb.dump
import ros2param.verb.get
import ros2param.verb.list
import ros2param.verb.load
import ros2param.verb.set

import ros2.ros2_cmd

COMMAND_EXTENSIONS = {
    'delete': ros2param.verb.delete.DeleteVerb(),
    'describe': ros2param.verb.describe.DescribeVerb(),
    'dump': ros2param.verb.dump.DumpVerb(),
    'get': ros2param.verb.get.GetVerb(),
    'list': ros2param.verb.list.ListVerb(),
    'load': ros2param.verb.load.LoadVerb(),
    'set': ros2param.verb.set.SetVerb(),
}

extension = ros2.ros2_cmd.Ros2CommandExtension(COMMAND_EXTENSIONS)
sys.exit(ros2cli.cli.main(extension=extension))
