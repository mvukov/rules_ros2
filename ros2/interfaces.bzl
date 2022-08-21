# Copyright 2021 Milan Vukov
#
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
""" ROS2 IDL handling.
"""

load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@rules_cc//cc:defs.bzl", "cc_library")

Ros2InterfaceInfo = provider(
    "Provides info for interface code generation.",
    fields = [
        "info",
        "deps",
    ],
)

def _ros2_interface_library_impl(ctx):
    return [
        DefaultInfo(files = depset(ctx.files.srcs)),
        Ros2InterfaceInfo(
            info = struct(
                srcs = ctx.files.srcs,
            ),
            deps = depset(
                direct = [dep[Ros2InterfaceInfo].info for dep in ctx.attr.deps],
                transitive = [
                    dep[Ros2InterfaceInfo].deps
                    for dep in ctx.attr.deps
                ],
            ),
        ),
    ]

ros2_interface_library = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".action", ".msg", ".srv"],
            mandatory = True,
        ),
        "deps": attr.label_list(providers = [Ros2InterfaceInfo]),
    },
    implementation = _ros2_interface_library_impl,
)

def _to_snake_case(not_snake_case):
    """ Converts camel-case to snake-case.

    Based on convert_camel_case_to_lower_case_underscore from rosidl_cmake.
    Unfortunately regex doesn't exist in Bazel.

    Args:
      not_snake_case: a camel-case string.
    Returns:
      A snake-case string.
    """
    result = ""
    not_snake_case_padded = " " + not_snake_case + " "
    for i in range(len(not_snake_case)):
        prev_char, char, next_char = not_snake_case_padded[i:i + 3].elems()
        if char.isupper() and next_char.islower() and prev_char != " ":
            # Insert an underscore before any upper case letter which is not
            # followed by another upper case letter.
            result += "_"
        elif char.isupper() and (prev_char.islower() or prev_char.isdigit()):
            # Insert an underscore before any upper case letter which is
            # preseded by a lower case letter or number.
            result += "_"
        result += char.lower()

    return result

def _get_stem(path):
    return path.basename[:-len(path.extension) - 1]

def _run_adapter(ctx, package_name, srcs):
    adapter_arguments = struct(
        package_name = package_name,
        non_idl_tuples = [":{}".format(src.path) for src in srcs],
    )

    adapter_arguments_file = ctx.actions.declare_file(
        "{}/rosidl_adapter_args.json".format(package_name),
    )
    ctx.actions.write(adapter_arguments_file, adapter_arguments.to_json())
    adapter_map = ctx.actions.declare_file(
        "{}/rosidl_adapter_map.idls".format(package_name),
    )
    output_dir = adapter_map.dirname

    idl_files = []
    idl_tuples = []
    for src in srcs:
        extension = src.extension
        stem = _get_stem(src)
        idl_files.append(ctx.actions.declare_file(
            "{}/{}/{}.idl".format(package_name, extension, stem),
        ))
        idl_tuples.append(
            "{}:{}/{}.idl".format(output_dir, extension, stem),
        )

    adapter_cmd_args = ctx.actions.args()
    adapter_cmd_args.add(package_name, format = "--package-name=%s")
    adapter_cmd_args.add(
        adapter_arguments_file,
        format = "--arguments-file=%s",
    )
    adapter_cmd_args.add(output_dir, format = "--output-dir=%s")
    adapter_cmd_args.add(adapter_map, format = "--output-file=%s")

    ctx.actions.run(
        inputs = srcs + [adapter_arguments_file],
        outputs = [adapter_map] + idl_files,
        executable = ctx.executable._adapter,
        arguments = [adapter_cmd_args],
    )

    return idl_files, idl_tuples

def _get_parent_dir(path):
    return "/".join(path.split("/")[:-1])

def _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        generator,
        generator_templates,
        output_mapping,
        visibility_control_template = None,
        extra_generator_args = None):
    generator_templates = generator_templates[DefaultInfo].files.to_list()

    generator_arguments_file = ctx.actions.declare_file(
        "{}/{}_args.json".format(package_name, generator.basename),
    )
    output_dir = generator_arguments_file.dirname
    generator_arguments = struct(
        package_name = package_name,
        idl_tuples = idl_tuples,
        output_dir = output_dir,
        template_dir = generator_templates[0].dirname,
        target_dependencies = [],  # TODO(mvukov) Do we need this?
    )
    ctx.actions.write(generator_arguments_file, generator_arguments.to_json())

    generator_cmd_args = ctx.actions.args()
    generator_cmd_args.add(
        generator_arguments_file.path,
        format = "--generator-arguments-file=%s",
    )
    if extra_generator_args:
        for arg in extra_generator_args:
            generator_cmd_args.add(arg)

    generator_outputs = []
    for src in srcs:
        extension = src.extension
        stem = _get_stem(src)
        snake_case_stem = _to_snake_case(stem)
        for t in output_mapping:
            relative_file = "{}/{}/{}".format(
                package_name,
                extension,
                t % snake_case_stem,
            )
            generator_outputs.append(ctx.actions.declare_file(relative_file))

    ctx.actions.run(
        inputs = idl_files + generator_templates + [generator_arguments_file],
        outputs = generator_outputs,
        executable = generator,
        arguments = [generator_cmd_args],
    )

    if visibility_control_template:
        visibility_control_basename = _get_stem(visibility_control_template)
        relative_file = "{}/msg/{}".format(
            package_name,
            visibility_control_basename,
        )
        visibility_control_h = ctx.actions.declare_file(relative_file)
        generator_outputs.append(visibility_control_h)
        ctx.actions.expand_template(
            template = visibility_control_template,
            output = visibility_control_h,
            substitutions = {
                "@PROJECT_NAME@": package_name,
                "@PROJECT_NAME_UPPER@": package_name.upper(),
            },
        )

    cc_include_dir = _get_parent_dir(output_dir)
    return generator_outputs, cc_include_dir

_INTERFACE_GENERATOR_C_OUTPUT_MAPPING = [
    "%s.h",
    "detail/%s__functions.c",
    "detail/%s__functions.h",
    "detail/%s__struct.h",
    "detail/%s__type_support.h",
]

_TYPESUPPORT_GENERATOR_C_OUTPUT_MAPPING = ["%s__type_support.c"]

_TYPESUPPORT_INTROSPECION_GENERATOR_C_OUTPUT_MAPPING = [
    "detail/%s__rosidl_typesupport_introspection_c.h",
    "detail/%s__type_support.c",
]

def _get_hdrs(files):
    return [f for f in files if f.path.endswith(".h") or f.path.endswith(".hpp")]

def _get_srcs(files):
    return [f for f in files if f.path.endswith(".c") or f.path.endswith(".cpp")]

def _c_generator_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs

    idl_files, idl_tuples = _run_adapter(
        ctx,
        package_name,
        srcs,
    )

    interface_outputs, cc_include_dir = _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        ctx.executable._interface_generator,
        ctx.attr._interface_templates,
        _INTERFACE_GENERATOR_C_OUTPUT_MAPPING,
        visibility_control_template = ctx.file._interface_visibility_control_template,
    )

    typesupport_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        ctx.executable._typesupport_generator,
        ctx.attr._typesupport_templates,
        _TYPESUPPORT_GENERATOR_C_OUTPUT_MAPPING,
        visibility_control_template = ctx.file._typesupport_visibility_control_template,
        extra_generator_args = [
            # TODO(mvukov) There are also rosidl_typesupport_connext_c and
            # rosidl_typesupport_fastrtps_c.
            "--typesupports=rosidl_typesupport_introspection_c",
        ],
    )

    typesupport_introspection_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        ctx.executable._typesupport_introspection_generator,
        ctx.attr._typesupport_introspection_templates,
        _TYPESUPPORT_INTROSPECION_GENERATOR_C_OUTPUT_MAPPING,
        visibility_control_template = ctx.file._typesupport_introspection_visibility_control_template,
    )

    # compilation_context_2, compilation_outputs = cc_common.compile()

    all_outputs = interface_outputs + typesupport_outputs + typesupport_introspection_outputs
    compilation_context = cc_common.create_compilation_context(
        headers = depset(_get_hdrs(all_outputs)),
        system_includes = depset([cc_include_dir]),
    )
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [
            CcInfo(compilation_context = compilation_context),
        ] + [
            dep[CcInfo]
            for dep in ctx.rule.attr.deps
        ],
    )
    return [cc_info]

c_generator_aspect = aspect(
    implementation = _c_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_adapter": attr.label(
            default = Label("@ros2_rosidl//:rosidl_adapter_app"),
            executable = True,
            cfg = "exec",
        ),
        "_interface_generator": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_c_app"),
            executable = True,
            cfg = "exec",
        ),
        "_interface_templates": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_c_templates"),
        ),
        "_interface_visibility_control_template": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in"),
            allow_single_file = True,
        ),
        "_typesupport_generator": attr.label(
            default = Label("@ros2_rosidl_typesupport//:rosidl_typesupport_generator_c_app"),
            executable = True,
            cfg = "exec",
        ),
        "_typesupport_templates": attr.label(
            default = Label("@ros2_rosidl_typesupport//:rosidl_typesupport_generator_c_templates"),
        ),
        "_typesupport_visibility_control_template": attr.label(
            default = Label("@ros2_rosidl_typesupport//:rosidl_typesupport_c/resource/rosidl_typesupport_c__visibility_control.h.in"),
            allow_single_file = True,
        ),
        "_typesupport_introspection_generator": attr.label(
            default = Label("@ros2_rosidl//:rosidl_typesupport_introspection_generator_c"),
            executable = True,
            cfg = "exec",
        ),
        "_typesupport_introspection_templates": attr.label(
            default = Label("@ros2_rosidl//:rosidl_typesupport_introspection_generator_c_templates"),
        ),
        "_typesupport_introspection_visibility_control_template": attr.label(
            default = Label("@ros2_rosidl//:rosidl_typesupport_introspection_c/resource/rosidl_typesupport_introspection_c__visibility_control.h.in"),
            allow_single_file = True,
        ),
    },
)

def _generator_impl(ctx):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [cc_info]

def _c_generator_impl(ctx):
    return _generator_impl(ctx)

c_generator = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [c_generator_aspect],
            providers = [Ros2InterfaceInfo],
        ),
    },
    implementation = _c_generator_impl,
)

def c_ros2_interface_library(name, deps, **kwargs):
    name_c = "{}_c".format(name)
    c_generator(
        name = name_c,
        deps = deps,
    )
    cc_library(
        name = name,
        deps = [
            name_c,
            "@ros2_rosidl//:rosidl_runtime_c",
            "@ros2_rosidl//:rosidl_typesupport_introspection_c",
            "@ros2_rosidl_typesupport//:rosidl_typesupport_c",
        ],
        copts = ["-std=c11"],
        **kwargs
    )

_INTERFACE_GENERATOR_CPP_OUTPUT_MAPPING = [
    "%s.hpp",
    "detail/%s__builder.hpp",
    "detail/%s__struct.hpp",
    "detail/%s__traits.hpp",
]

_TYPESUPPORT_GENERATOR_CPP_OUTPUT_MAPPING = [
    "%s__type_support.cpp",
]

_TYPESUPPORT_INTROSPECION_GENERATOR_CPP_OUTPUT_MAPPING = [
    "detail/%s__rosidl_typesupport_introspection_cpp.hpp",
    "detail/%s__type_support.cpp",
]

def _cpp_generator_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs

    idl_files, idl_tuples = _run_adapter(
        ctx,
        package_name,
        srcs,
    )

    interface_outputs, cc_include_dir = _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        ctx.executable._interface_generator,
        ctx.attr._interface_templates,
        _INTERFACE_GENERATOR_CPP_OUTPUT_MAPPING,
    )

    typesupport_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        ctx.executable._typesupport_generator,
        ctx.attr._typesupport_templates,
        _TYPESUPPORT_GENERATOR_CPP_OUTPUT_MAPPING,
        extra_generator_args = [
            # TODO(mvukov) There are also rosidl_typesupport_connext_cpp and
            # rosidl_typesupport_fastrtps_cpp.
            "--typesupports=rosidl_typesupport_introspection_cpp",
        ],
    )

    typesupport_introspection_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        idl_files,
        idl_tuples,
        ctx.executable._typesupport_introspection_generator,
        ctx.attr._typesupport_introspection_templates,
        _TYPESUPPORT_INTROSPECION_GENERATOR_CPP_OUTPUT_MAPPING,
    )

    all_outputs = interface_outputs + typesupport_outputs + typesupport_introspection_outputs
    compilation_context = cc_common.create_compilation_context(
        headers = depset(_get_hdrs(all_outputs)),
        system_includes = depset([cc_include_dir]),
    )
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [
            CcInfo(compilation_context = compilation_context),
        ] + [
            dep[CcInfo]
            for dep in ctx.rule.attr.deps
        ],
    )
    return [cc_info]

cpp_generator_aspect = aspect(
    implementation = _cpp_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_adapter": attr.label(
            default = Label("@ros2_rosidl//:rosidl_adapter_app"),
            executable = True,
            cfg = "exec",
        ),
        "_interface_generator": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_cpp_app"),
            executable = True,
            cfg = "exec",
        ),
        "_interface_templates": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_cpp_templates"),
        ),
        "_typesupport_generator": attr.label(
            default = Label("@ros2_rosidl_typesupport//:rosidl_typesupport_generator_cpp_app"),
            executable = True,
            cfg = "exec",
        ),
        "_typesupport_templates": attr.label(
            default = Label("@ros2_rosidl_typesupport//:rosidl_typesupport_generator_cpp_templates"),
        ),
        "_typesupport_introspection_generator": attr.label(
            default = Label("@ros2_rosidl//:rosidl_typesupport_introspection_generator_cpp"),
            executable = True,
            cfg = "exec",
        ),
        "_typesupport_introspection_templates": attr.label(
            default = Label("@ros2_rosidl//:rosidl_typesupport_introspection_generator_cpp_templates"),
        ),
    },
)

def _cpp_generator_impl(ctx):
    return _generator_impl(ctx)

cpp_generator = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [cpp_generator_aspect],
            providers = [Ros2InterfaceInfo],
        ),
    },
    implementation = _cpp_generator_impl,
)

def cpp_ros2_interface_library(name, deps, **kwargs):
    name_cpp = "{}_cpp".format(name)
    cpp_generator(
        name = name_cpp,
        deps = deps,
    )
    cc_library(
        name = name,
        copts = ["-std=c++14"],
        deps = [
            name_cpp,
            "@ros2_rosidl//:rosidl_runtime_cpp",
            "@ros2_rosidl//:rosidl_typesupport_introspection_c",
            "@ros2_rosidl//:rosidl_typesupport_introspection_cpp",
            "@ros2_rosidl_typesupport//:rosidl_typesupport_cpp",
        ],
        **kwargs
    )
