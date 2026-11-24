""" Generates version headers from package.xml files.
"""

_GENERATOR = "@com_github_mvukov_rules_ros2//repositories:generate_version_header"

def ament_version_header(name, package_xml, out, **kwargs):
    """ Generates a version header equivalent to ament_cmake_gen_version_h.

    The package name and version are read from the given package.xml.

    Mirrors https://github.com/ament/ament_cmake/blob/rolling/ament_cmake_gen_version_h/cmake/version.h.in

    Args:
        name: A unique target name.
        package_xml: The package.xml of the package, e.g. "rclcpp/package.xml".
        out: The output header path.
        **kwargs: Common rule arguments, passed to the underlying genrule.
    """
    native.genrule(
        name = name,
        srcs = [package_xml],
        outs = [out],
        cmd = "$(execpath {}) $(execpath {}) $@".format(_GENERATOR, package_xml),
        tools = [_GENERATOR],
        **kwargs
    )

def package_version_template(name, package_xml, template, out, substitutions, **kwargs):
    """ Expands a version template with the version from a package.xml.

    Every substitution key must be present in the template, and no
    CMake-style @PLACEHOLDER@ may remain in the output.

    Args:
        name: A unique target name.
        package_xml: The package.xml of the package.
        template: The template to expand, e.g. "include/foo/version.hpp.in".
        out: The output file path.
        substitutions: A dict mapping template keys to values. Values may use
            the {name}, {version}, {major}, {minor} and {patch} fields, which
            are read from the package.xml.
        **kwargs: Common rule arguments, passed to the underlying genrule.
    """
    args = " ".join([
        "--substitution '{}={}'".format(key, value)
        for key, value in substitutions.items()
    ])
    native.genrule(
        name = name,
        srcs = [package_xml, template],
        outs = [out],
        cmd = "$(execpath {}) $(execpath {}) $@ --template $(execpath {}) {}".format(
            _GENERATOR,
            package_xml,
            template,
            args,
        ),
        tools = [_GENERATOR],
        **kwargs
    )
