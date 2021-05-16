# Copyright 2021 Milan Vukov

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#    http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
""" ROS2 IDL handling.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

RosIdlInfo = provider("Provides info for IDL code generation.", fields = [
    "info",
    "deps",
])

def _ros_idl_library_impl(ctx):
    package_name = ctx.attr.package_name
    if not package_name:
        package_name = ctx.label.name

    return [
        DefaultInfo(files = depset(ctx.files.srcs)),
        RosIdlInfo(
            info = struct(
                package_name = package_name,
                srcs = ctx.files.srcs,
            ),
            deps = depset(
                direct = [dep[RosIdlInfo].info for dep in ctx.attr.deps],
                transitive = [dep[RosIdlInfo].deps for dep in ctx.attr.deps],
            ),
        ),
    ]

ros_idl_library = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".action", ".msg", ".srv"],
            mandatory = True,
        ),
        "deps": attr.label_list(providers = [RosIdlInfo]),
        "package_name": attr.string(),
    },
    implementation = _ros_idl_library_impl,
)

RosIdlCGeneratorAspectInfo = provider("TBD", fields = [
    "idl_files",
    "c_files",
])

_C_GENERATOR_OUTPUT_TEMPLATES = [
    "%s.h",
    "detail/%s__functions.c",
    "detail/%s__functions.h",
    "detail/%s__struct.h",
    "detail/%s__type_support.h",
]

def to_snake_case(not_snake_case):
    """ https://stackoverflow.com/a/19940888
    """
    final = ""
    for i in range(len(not_snake_case)):
        item = not_snake_case[i]
        if i < len(not_snake_case) - 1:
            next_char_will_be_underscored = (
                not_snake_case[i + 1] == "_" or
                not_snake_case[i + 1] == " " or
                not_snake_case[i + 1].isupper()
            )
        if (item == " " or item == "_") and next_char_will_be_underscored:
            continue
        elif (item == " " or item == "_"):
            final += "_"
        elif item.isupper():
            final += "_" + item.lower()
        else:
            final += item
    if final[0] == "_":
        final = final[1:]
    return final

def _get_visibility_control_template(templates):
    for template in templates:
        if template.basename == "rosidl_generator_c__visibility_control.h.in":
            return template
    return None

def _c_generator_aspect_impl(target, ctx):
    target_name = target.label.name
    info = target[RosIdlInfo].info
    package_name = info.package_name
    srcs = info.srcs
    deps = target[RosIdlInfo].deps

    adapter_arguments = struct(
        package_name = target_name,
        non_idl_tuples = [":{}".format(src.path) for src in srcs],
    )

    relative_dir = "{}".format(target_name)
    adapter_arguments_file = ctx.actions.declare_file(
        "{}/rosidl_adapter_args.json".format(relative_dir),
    )
    ctx.actions.write(adapter_arguments_file, adapter_arguments.to_json())
    adapter_map = ctx.actions.declare_file(
        "{}/rosidl_adapter_map.idls".format(relative_dir),
    )
    output_dir = adapter_map.dirname

    idl_files = []
    for src in srcs:
        extension = src.extension
        stem = src.basename[:-len(extension) - 1]
        idl_files.append(ctx.actions.declare_file(
            "{}/{}/{}.idl".format(relative_dir, extension, stem),
        ))

    adapter_cmd_args = ctx.actions.args()
    adapter_cmd_args.add(package_name, format = "--package-name=%s")
    adapter_cmd_args.add(
        adapter_arguments_file.path,
        format = "--arguments-file=%s",
    )
    adapter_cmd_args.add(output_dir, format = "--output-dir=%s")
    adapter_cmd_args.add(adapter_map.path, format = "--output-file=%s")

    ctx.actions.run(
        inputs = srcs + [adapter_arguments_file],
        outputs = [adapter_map] + idl_files,
        executable = ctx.executable._adapter,
        arguments = [adapter_cmd_args],
    )

    generator_idl_tuples = []
    for src in srcs:
        extension = src.extension
        stem = src.basename[:-len(extension) - 1]
        generator_idl_tuples.append(
            "{}:{}/{}.idl".format(output_dir, extension, stem),
        )

    templates = ctx.attr._templates[DefaultInfo].files.to_list()
    template_dir = templates[0].dirname

    generator_arguments = struct(
        package_name = package_name,
        idl_tuples = generator_idl_tuples,
        ros_interface_dependencies = [],  # TODO(mvukov) Do we need this?
        output_dir = output_dir,
        template_dir = template_dir,
        target_dependencies = [],
    )
    generator_arguments_file = ctx.actions.declare_file(
        "{}/rosidl_generator_c_args.json".format(relative_dir),
    )
    ctx.actions.write(generator_arguments_file, generator_arguments.to_json())

    generator_cmd_args = ctx.actions.args()
    generator_cmd_args.add(
        generator_arguments_file.path,
        format = "--generator-arguments-file=%s",
    )

    generator_outputs = []
    for src in srcs:
        extension = src.extension
        stem = src.basename[:-len(extension) - 1]
        snake_case_stem = to_snake_case(stem)
        for t in _C_GENERATOR_OUTPUT_TEMPLATES:
            generator_outputs.append(ctx.actions.declare_file(
                "{}/{}/{}".format(relative_dir, extension, t % snake_case_stem),
            ))

    ctx.actions.run(
        inputs = idl_files + templates + [generator_arguments_file],
        outputs = generator_outputs,
        executable = ctx.executable._generator,
        arguments = [generator_cmd_args],
    )

    visibility_control_h = ctx.actions.declare_file(
        "{}/msg/rosidl_generator_c__visibility_control.h".format(relative_dir),
    )
    ctx.actions.expand_template(
        template = _get_visibility_control_template(templates),
        output = visibility_control_h,
        substitutions = {
            "@PROJECT_NAME@": target_name,
            "@PROJECT_NAME_UPPER@": target_name.upper(),
        },
    )

    c_files = generator_outputs + [visibility_control_h]
    for dep in ctx.rule.attr.deps:
        idl_files += dep[RosIdlCGeneratorAspectInfo].idl_files
        c_files += dep[RosIdlCGeneratorAspectInfo].c_files

    return [
        RosIdlCGeneratorAspectInfo(
            idl_files = idl_files,
            c_files = c_files,
        ),
    ]

c_generator_aspect = aspect(
    implementation = _c_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_adapter": attr.label(
            default = Label("@ros2_rosidl//:rosidl_adapter_app"),
            executable = True,
            cfg = "exec",
        ),
        "_generator": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_c_app"),
            executable = True,
            cfg = "exec",
        ),
        "_templates": attr.label(
            default = Label("@ros2_rosidl//:rosidl_generator_c_templates"),
        ),
    },
)

def _c_ros_idl_compile_impl(ctx):
    files = depset(
        transitive = [
            depset(dep[RosIdlCGeneratorAspectInfo].c_files)
            for dep in ctx.attr.deps
        ],
    )
    return [DefaultInfo(files = files)]

c_ros_idl_compile = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [c_generator_aspect],
            providers = [RosIdlInfo],
        ),
    },
    implementation = _c_ros_idl_compile_impl,
)

def c_ros_idl_library(name, deps, visibility = None):
    name_c = "{}_rosidl_generator_c".format(name)
    c_ros_idl_compile(
        name = name_c,
        deps = deps,
    )
    cc_library(
        name = name,
        srcs = [name_c],
        deps = [
            "@ros2_rosidl//:rosidl_runtime_c",
        ],
        visibility = visibility,
    )
