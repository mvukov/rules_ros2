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
import sys

import ros2cli.cli
import ros2topic.verb.bw
import ros2topic.verb.delay
import ros2topic.verb.echo
import ros2topic.verb.find
import ros2topic.verb.hz
import ros2topic.verb.info
import ros2topic.verb.list
import ros2topic.verb.pub
import ros2topic.verb.type

import ros2.ros2_cmd

COMMAND_EXTENSIONS = {
    'bw': ros2topic.verb.bw.BwVerb(),
    'delay': ros2topic.verb.delay.DelayVerb(),
    'echo': ros2topic.verb.echo.EchoVerb(),
    'find': ros2topic.verb.find.FindVerb(),
    'hz': ros2topic.verb.hz.HzVerb(),
    'info': ros2topic.verb.info.InfoVerb(),
    'list': ros2topic.verb.list.ListVerb(),
    'pub': ros2topic.verb.pub.PubVerb(),
    'type': ros2topic.verb.type.TypeVerb(),
}

extension = ros2.ros2_cmd.Ros2CommandExtension(COMMAND_EXTENSIONS)
sys.exit(ros2cli.cli.main(argv=sys.argv[1:], extension=extension))
