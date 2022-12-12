import sys

import ros2.ros2_cmd
import ros2bag.verb.convert
import ros2bag.verb.info
import ros2bag.verb.list
import ros2bag.verb.play
import ros2bag.verb.record
import ros2bag.verb.reindex
import ros2cli.cli

import {ament_setup}

{ament_setup}.set_up_ament()

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
