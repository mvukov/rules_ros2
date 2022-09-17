# Copyright 2016-2017 Dirk Thomas
# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import argparse
import sys

import ros2cli.command
import ros2cli.entry_points
from ros2cli import cli

import ros2topic.verb.bw
import ros2topic.verb.delay
import ros2topic.verb.echo
import ros2topic.verb.find
import ros2topic.verb.hz
import ros2topic.verb.info
import ros2topic.verb.list
import ros2topic.verb.pub
import ros2topic.verb.type

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

# add_subparsers and TopicCommand are based on code from the ros2cli repo
# at commit 7f36483ad48c73105b450bb9404a13889e01d442.


def add_subparsers(parser,
                   cli_name,
                   dest,
                   command_extensions,
                   hide_extensions=None,
                   required=True):
    """
    Create argparse subparser for each extension.

    The ``cli_name`` is used for the title and description of the
    ``add_subparsers`` function call.

    For each extension a subparser is created.
    If the extension has an ``add_arguments`` method it is being called.

    This method is deprecated.
    Use the function ``add_subparsers_on_demand`` instead.
    Their signatures are almost identical.
    Instead of passing the extensions the new function expects the group name
    of these extensions.

    :param parser: the parent argument parser
    :type parser: :py:class:`argparse.ArgumentParser`
    :param str cli_name: name of the command line command to which the
      subparsers are being added
    :param str dest: name of the attribute under which the selected extension
      will be stored
    :param dict command_extensions: dict of command extensions by their name
      where each contributes a command with specific arguments
    """
    # add subparser with description of available subparsers
    description = ''
    if command_extensions:
        max_length = max(
            len(name)
            for name in command_extensions.keys()
            if hide_extensions is None or name not in hide_extensions)
        for name in sorted(command_extensions.keys()):
            if hide_extensions is not None and name in hide_extensions:
                continue
            extension = command_extensions[name]
            description += '%s  %s\n' % (name.ljust(
                max_length), ros2cli.entry_points.get_first_line_doc(extension))
    subparser = parser.add_subparsers(
        title='Commands',
        description=description,
        metavar=f'Call `{cli_name} <command> -h` for more detailed usage.')
    # use a name which doesn't collide with any argument
    # but is readable when shown as part of the the usage information
    subparser.dest = ' ' + dest.lstrip('_')
    subparser.required = required

    # add extension specific sub-parser with its arguments
    for name in sorted(command_extensions.keys()):
        extension = command_extensions[name]
        command_parser = subparser.add_parser(
            name,  # NOTE: Here was a bug, replaced extension.NAME with name.
            description=ros2cli.entry_points.get_first_line_doc(extension),
            formatter_class=argparse.RawDescriptionHelpFormatter)
        command_parser.set_defaults(**{dest: extension})
        if hasattr(extension, 'add_arguments'):
            extension.add_arguments(command_parser, f'{cli_name} {name}')

    return subparser


class TopicCommand(ros2cli.command.CommandExtension):
    """Various topic related sub-commands."""

    def add_arguments(self, parser, cli_name):
        # self._subparser = parser
        parser.add_argument('--include-hidden-topics',
                            action='store_true',
                            help='Consider hidden topics as well')

        # add arguments and sub-commands of verbs
        add_subparsers(parser,
                       cli_name,
                       '_verb',
                       COMMAND_EXTENSIONS,
                       required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            parser.print_help()
            return 0

        extension = getattr(args, '_verb')
        return extension.main(args=args)


extension = TopicCommand()
sys.exit(cli.main(argv=sys.argv[1:], extension=extension))
