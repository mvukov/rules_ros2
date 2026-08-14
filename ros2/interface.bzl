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

""" Implements a macro for setting up a ROS 2 interface app.
"""

load("//ros2:py_defs.bzl", "ros2_py_binary")


def ros2_interface(name, deps, **kwargs):
    """ Defines a ROS 2 interface app for a set of deps.

    Args:
        name: A unique target name.
        deps: A list of Python targets: typically IDL libraries
        (py_ros2_interface_library targets) used by the topic app.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """

    ros2_py_binary(
        name = name,
        srcs = ["@com_github_mvukov_rules_ros2//ros2:ros2_interface.py"],
        main = "@com_github_mvukov_rules_ros2//ros2:ros2_interface.py",
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2_rcl_interfaces//:py_action_msgs",
            "@ros2_rcl_interfaces//:py_builtin_interfaces",
            "@ros2_rcl_interfaces//:py_rcl_interfaces",
            "@ros2cli",
            "@ros2cli//:ros2interface",
            "@ros2_rosidl//:rosidl_adapter_lib",
        ] + deps,
        set_up_ament = True,
        **kwargs
    )
