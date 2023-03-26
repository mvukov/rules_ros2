""" Defines commonly used C/C++ macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "sh_launcher")
load("@com_github_mvukov_rules_ros2//ros2:cc_opts.bzl", "CPP_COPTS", "C_COPTS")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

def _ros2_cc_target(target, lang, name, ros2_package_name, **kwargs):
    if lang == "c":
        all_copts = C_COPTS
    elif lang == "cpp":
        all_copts = CPP_COPTS
    else:
        fail("lang must be set to c or cpp!")
    all_copts = all_copts + kwargs.pop("copts", [])

    ros2_package_name = ros2_package_name or name
    all_local_defines = ["ROS_PACKAGE_NAME=\\\"{}\\\"".format(ros2_package_name)]
    all_local_defines = all_local_defines + kwargs.pop("local_defines", [])

    target(
        name = name,
        copts = all_copts,
        local_defines = all_local_defines,
        **kwargs
    )

def ros2_c_library(name, ros2_package_name = None, **kwargs):
    """ Defines a ROS 2 C library.

    Adds common ROS 2 C definitions on top of a cc_library.

    Args:
        name: A unique target name.
        ros2_package_name: If given, defines a ROS 2 package name for the target.
            Otherwise, the `name` is used as the package name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    _ros2_cc_target(cc_library, "c", name, ros2_package_name, **kwargs)

def ros2_cpp_library(name, ros2_package_name = None, **kwargs):
    """ Defines a ROS 2 C++ library.

    Adds common ROS 2 C++ definitions on top of a cc_library.

    Args:
        name: A unique target name.
        ros2_package_name: If given, defines a ROS 2 package name for the target.
            Otherwise, the `name` is used as the package name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    _ros2_cc_target(cc_library, "cpp", name, ros2_package_name, **kwargs)

def ros2_c_binary(name, ros2_package_name = None, **kwargs):
    """ Defines a ROS 2 C binary.

    Adds common ROS 2 C definitions on top of a cc_binary.

    Args:
        name: A unique target name.
        ros2_package_name: If given, defines a ROS package name for the target.
            Otherwise, the `name` is used as the package name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    _ros2_cc_target(cc_binary, "c", name, ros2_package_name, **kwargs)

def _ros2_cpp_exec(target, name, ros2_package_name = None, set_up_ament = False, **kwargs):
    if set_up_ament == False:
        _ros2_cc_target(target, "cpp", name, ros2_package_name, **kwargs)
        return

    target_impl = name + "_impl"
    tags = kwargs.pop("tags", [])
    _ros2_cc_target(cc_binary, "cpp", target_impl, ros2_package_name, tags = ["manual"], **kwargs)

    is_test = target == cc_test

    launcher = "{}_launch".format(name)
    sh_launcher(
        launcher,
        deps = [target_impl],
        template = "@com_github_mvukov_rules_ros2//ros2:launch.sh.tpl",
        substitutions = {
            "{entry_point}": "$(rootpath {})".format(target_impl),
        },
        tags = ["manual"],
        data = [target_impl],
        testonly = is_test,
    )

    sh_target = native.sh_test if is_test else native.sh_binary
    sh_target(
        name = name,
        srcs = [launcher],
        data = [target_impl],
        tags = tags,
        visibility = kwargs.pop("visibility", None),
    )

def ros2_cpp_binary(name, ros2_package_name = None, set_up_ament = False, **kwargs):
    """ Defines a ROS 2 C++ binary.

    Adds common ROS 2 C++ definitions on top of a cc_binary.

    Args:
        name: A unique target name.
        ros2_package_name: If given, defines a ROS package name for the target.
            Otherwise, the `name` is used as the package name.
        set_up_ament: If true, sets up ament file tree for the binary target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    _ros2_cpp_exec(cc_binary, name, ros2_package_name, set_up_ament, **kwargs)

def ros2_cpp_test(name, ros2_package_name = None, set_up_ament = False, **kwargs):
    """ Defines a ROS 2 C++ test.

    Adds common ROS 2 C++ definitions on top of a cc_test.

    Args:
        name: A unique target name.
        ros2_package_name: If given, defines a ROS package name for the target.
            Otherwise, the `name` is used as the package name.
        set_up_ament: If true, sets up ament file tree for the test target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    _ros2_cpp_exec(cc_test, name, ros2_package_name, set_up_ament, **kwargs)
