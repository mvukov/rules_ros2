""" Defines commonly used C/C++ macros2.
"""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

C_COPTS = ["-std=c11"]
CPP_COPTS = ["-std=c++17"]

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

def ros2_cpp_binary(name, ros2_package_name = None, **kwargs):
    """ Defines a ROS 2 C++ binary.

    Adds common ROS 2 C++ definitions on top of a cc_binary.

    Args:
        name: A unique target name.
        ros2_package_name: If given, defines a ROS package name for the target.
            Otherwise, the `name` is used as the package name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    _ros2_cc_target(cc_binary, "cpp", name, ros2_package_name, **kwargs)
