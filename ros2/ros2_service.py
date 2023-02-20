# Copyright 2023 Dennis Bruggner
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
import sys

import ros2cli.cli
import ros2service.verb.call
import ros2service.verb.find
import ros2service.verb.list
import ros2service.verb.type

import ros2.ros2_cmd

COMMAND_EXTENSIONS = {
    'call': ros2service.verb.call.CallVerb(),
    'find': ros2service.verb.find.FindVerb(),
    'list': ros2service.verb.list.ListVerb(),
    'type': ros2service.verb.type.TypeVerb(),
}

extension = ros2.ros2_cmd.Ros2CommandExtension(COMMAND_EXTENSIONS)
sys.exit(ros2cli.cli.main(extension=extension))
