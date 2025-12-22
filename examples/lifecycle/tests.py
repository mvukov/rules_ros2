# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import re
import unittest

import launch.actions
import launch.event_handlers.on_process_start
import launch_ros.actions
import launch_ros.events.lifecycle
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import lifecycle_msgs.msg
import python.runfiles

_Transition = lifecycle_msgs.msg.Transition

TALKER_NODE_PATH = python.runfiles.Runfiles.Create().Rlocation(
    'com_github_mvukov_rules_ros2/examples/lifecycle/lifecycle_talker')
LISTENER_NODE_PATH = python.runfiles.Runfiles.Create().Rlocation(
    'com_github_mvukov_rules_ros2/examples/lifecycle/lifecycle_listener')


@launch_testing.markers.keep_alive
def generate_test_description():
    talker_node = launch_ros.actions.LifecycleNode(
        executable=TALKER_NODE_PATH,
        name='lc_talker',
        namespace='',
        output='screen')
    listener_node = launch_ros.actions.Node(
        executable=LISTENER_NODE_PATH,
        name='listener',
        output='screen')

    def emit_change_state_event(transition_id):
        return launch.actions.EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(
                    talker_node),
                transition_id=transition_id))

    return launch.LaunchDescription([
        talker_node,
        listener_node,
        # Right after the talker starts, make it take the 'configure'
        # transition.
        launch.actions.RegisterEventHandler(
            launch.event_handlers.on_process_start.OnProcessStart(
                target_action=talker_node,
                on_start=[
                    launch.actions.TimerAction(
                        period=2.0,
                        actions=[
                            emit_change_state_event(
                                _Transition.TRANSITION_CONFIGURE)
                        ]),
                ],
            )),
        # When the talker reaches the 'inactive' state, make it take the
        # 'activate' transition.
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    emit_change_state_event(_Transition.TRANSITION_ACTIVATE)
                ],
            )),
        # When the talker node reaches the 'active' state, wait a bit and then
        # make it take the 'deactivate' transition.
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node,
                start_state='activating',
                goal_state='active',
                entities=[
                    launch.actions.TimerAction(
                        period=0.5,
                        actions=[
                            emit_change_state_event(
                                _Transition.TRANSITION_DEACTIVATE)
                        ]),
                ],
            )),
        # When the talker node reaches the 'inactive' state coming from
        # the 'active' state, make it take the 'cleanup' transition.
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node,
                start_state='deactivating',
                goal_state='inactive',
                entities=[
                    emit_change_state_event(_Transition.TRANSITION_CLEANUP)
                ],
            )),
        # When the talker node reaches the 'unconfigured' state after
        # a 'cleanup' transition, make it take the 'unconfigured_shutdown'
        # transition.
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=talker_node,
                start_state='cleaningup',
                goal_state='unconfigured',
                entities=[
                    emit_change_state_event(
                        _Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)
                ],
            )),
        launch_testing.actions.ReadyToTest()
    ]), {
        'talker_node': talker_node,
        'listener_node': listener_node,
    }


class TestLifecyclePubSub(unittest.TestCase):

    def test_talker_lifecycle(self, proc_info, proc_output, talker_node,
                              listener_node):
        """Test lifecycle talker."""
        proc_output.assertWaitFor('on_configure() is called',
                                  process=talker_node,
                                  timeout=15)
        proc_output.assertWaitFor('on_activate() is called',
                                  process=talker_node,
                                  timeout=15)
        pattern = re.compile(r'data_callback: Lifecycle HelloWorld #\d+')
        proc_output.assertWaitFor(expected_output=pattern,
                                  process=listener_node,
                                  timeout=15)
        proc_output.assertWaitFor('on_deactivate() is called',
                                  process=talker_node,
                                  timeout=15)
        proc_output.assertWaitFor('on cleanup is called',
                                  process=talker_node,
                                  timeout=15)
        proc_output.assertWaitFor('on shutdown is called',
                                  process=talker_node,
                                  timeout=15)


@launch_testing.post_shutdown_test()
class TestLifecyclePubSubAfterShutdown(unittest.TestCase):

    def test_talker_graceful_shutdown(self, proc_info, talker_node):
        """Test lifecycle talker graceful shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info, process=talker_node)
