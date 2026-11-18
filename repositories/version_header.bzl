""" Generates <package>/version.h like ament_generate_version_header().
"""

load("@bazel_skylib//rules:write_file.bzl", "write_file")

def ament_version_header(name, out, package_name, version):
    """ Generates a version header equivalent to ament_cmake_gen_version_h.

    Mirrors https://github.com/ament/ament_cmake/blob/rolling/ament_cmake_gen_version_h/cmake/version.h.in

    Args:
        name: A unique target name.
        out: The output header path.
        package_name: The ROS package name, e.g. rclcpp.
        version: The package version, e.g. "32.0.2". Keep it in sync with
            the version of the package.
    """
    major, minor, patch = version.split(".")
    p = package_name.upper()
    write_file(
        name = name,
        out = out,
        content = [
            "#ifndef {}__VERSION_H_".format(p),
            "#define {}__VERSION_H_".format(p),
            "#define {}_VERSION_MAJOR ({})".format(p, major),
            "#define {}_VERSION_MINOR ({})".format(p, minor),
            "#define {}_VERSION_PATCH ({})".format(p, patch),
            "#define {}_VERSION_STR \"{}\"".format(p, version),
            "#define {}_VERSION_GTE(major, minor, patch) ( \\".format(p),
            "     (major < {}_VERSION_MAJOR) ? true \\".format(p),
            "     : ((major > {}_VERSION_MAJOR) ? false \\".format(p),
            "     : ((minor < {}_VERSION_MINOR) ? true \\".format(p),
            "     : ((minor > {}_VERSION_MINOR) ? false \\".format(p),
            "     : ((patch < {}_VERSION_PATCH) ? true \\".format(p),
            "     : ((patch > {}_VERSION_PATCH) ? false \\".format(p),
            "     : true))))))",
            "#endif  // {}__VERSION_H_".format(p),
        ],
    )
