# Copyright 2023 Laurenz Altenmueller
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

sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)  # noqa

import ros2cli.cli  # noqa: E402
import ros2node.verb.info  # noqa: E402
import ros2node.verb.list  # noqa: E402

import ros2.ros2_cmd  # noqa: E402

COMMAND_EXTENSIONS = {
    'info': ros2node.verb.info.InfoVerb(),
    'list': ros2node.verb.list.ListVerb(),
}

extension = ros2.ros2_cmd.Ros2CommandExtension(COMMAND_EXTENSIONS)
sys.exit(ros2cli.cli.main(argv=sys.argv[1:], extension=extension))
