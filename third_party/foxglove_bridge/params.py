# Copyright 2023 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Extracted from ros2_foxglove_bridge/launch/foxglove_bridge_launch.xml  # noreorder  # noqa
PARAMS_TO_DEFAULT_VALUES = {
    'port':
        8765,
    'address':
        '0.0.0.0',
    'tls':
        False,
    'certfile':
        '',
    'keyfile':
        '',
    'topic_whitelist': ['.*'],
    'param_whitelist': ['.*'],
    'service_whitelist': ['.*'],
    'client_topic_whitelist': ['.*'],
    'max_qos_depth':
        10,
    'num_threads':
        0,
    'send_buffer_limit':
        10000000,
    'use_sim_time':
        False,
    'capabilities': [
        'clientPublish',
        'connectionGraph',
        'parameters',
        'parametersSubscribe',
        # For services we need auto-generated .msg files. TBD.
        # 'services',
    ],
}
