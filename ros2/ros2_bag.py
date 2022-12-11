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

import ros2bag.verb.convert
import ros2bag.verb.info
import ros2bag.verb.list
import ros2bag.verb.play
import ros2bag.verb.record
import ros2bag.verb.reindex
import ros2cli.cli

import ros2.ros2_cmd

COMMAND_EXTENSIONS = {
    'convert': ros2bag.verb.convert.ConvertVerb(),
    'info': ros2bag.verb.info.InfoVerb(),
    'list': ros2bag.verb.list.ListVerb(),
    'play': ros2bag.verb.play.PlayVerb(),
    'record': ros2bag.verb.record.RecordVerb(),
    'reindex': ros2bag.verb.reindex.ReindexVerb(),
}

extension = ros2.ros2_cmd.Ros2CommandExtension(COMMAND_EXTENSIONS)
sys.exit(ros2cli.cli.main(extension=extension))
