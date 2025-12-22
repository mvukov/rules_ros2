import sys

import ros2action.verb.info
import ros2action.verb.list
import ros2action.verb.send_goal
import ros2cli.cli

import ros2.ros2_cmd

COMMAND_EXTENSIONS = {
    'info': ros2action.verb.info.InfoVerb(),
    'list': ros2action.verb.list.ListVerb(),
    'send_goal': ros2action.verb.send_goal.SendGoalVerb(),
}

extension = ros2.ros2_cmd.Ros2CommandExtension(COMMAND_EXTENSIONS)
sys.exit(ros2cli.cli.main(extension=extension))
