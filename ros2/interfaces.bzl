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
""" ROS 2 IDL handling.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_github_mvukov_rules_ros2//ros2:cc_opts.bzl", "CPP_COPTS", "C_COPTS")
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")
load("@rules_python//python:defs.bzl", "py_library")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

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
    provides = [Ros2InterfaceInfo],
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
        mnemonic = "Ros2IdlAdapter",
        progress_message = "Generating IDL files for %{label}",
    )

    return idl_files, idl_tuples

IdlAdapterAspectInfo = provider("TBD", fields = [
    "idl_files",
    "idl_tuples",
])

def _idl_adapter_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs
    idl_files, idl_tuples = _run_adapter(ctx, package_name, srcs)
    return [
        IdlAdapterAspectInfo(
            idl_files = idl_files,
            idl_tuples = idl_tuples,
        ),
    ]

idl_adapter_aspect = aspect(
    implementation = _idl_adapter_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_adapter": attr.label(
            default = Label("@ros2_rosidl//:rosidl_adapter_app"),
            executable = True,
            cfg = "exec",
        ),
    },
    provides = [IdlAdapterAspectInfo],
)

def _get_parent_dir(path):
    return "/".join(path.split("/")[:-1])

def _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        generator,
        generator_templates,
        output_mapping,
        visibility_control_template = None,
        extra_generator_args = None,
        extra_generated_outputs = None,
        mnemonic = None,
        progress_message = None):
    generator_templates = generator_templates[DefaultInfo].files.to_list()

    generator_arguments_file = ctx.actions.declare_file(
        "{}/{}_args.json".format(package_name, generator.basename),
    )
    output_dir = generator_arguments_file.dirname
    generator_arguments = struct(
        package_name = package_name,
        idl_tuples = adapter.idl_tuples,
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

    extra_generated_outputs = extra_generated_outputs or []
    for extra_output in extra_generated_outputs:
        relative_file = "{}/{}".format(package_name, extra_output)
        generator_outputs.append(ctx.actions.declare_file(relative_file))

    ctx.actions.run(
        inputs = adapter.idl_files + generator_templates + [generator_arguments_file],
        outputs = generator_outputs,
        executable = generator,
        arguments = [generator_cmd_args],
        mnemonic = mnemonic,
        progress_message = progress_message,
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

CGeneratorAspectInfo = provider("TBD", fields = [
    "cc_info",
])

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
    return [
        f
        for f in files
        if f.path.endswith(".h") or f.path.endswith(".hpp")
    ]

def _get_srcs(files):
    return [
        f
        for f in files
        if f.path.endswith(".c") or f.path.endswith(".cpp")
    ]

def _get_compilation_contexts_from_deps(deps):
    return [dep[CcInfo].compilation_context for dep in deps]

def _get_compilation_contexts_from_aspect_info_deps(deps, aspect_info):
    return [dep[aspect_info].cc_info.compilation_context for dep in deps]

def _get_linking_contexts_from_deps(deps):
    return [dep[CcInfo].linking_context for dep in deps]

def _get_linking_contexts_from_aspect_info_deps(deps, aspect_info):
    return [dep[aspect_info].cc_info.linking_context for dep in deps]

def _compile_cc_generated_code(
        ctx,
        name,
        aspect_info,
        srcs,
        hdrs,
        deps,
        cc_include_dir,
        copts,
        target = None):
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    rule_deps = []
    rule_deps.extend(ctx.rule.attr.deps)
    if target != None:
        rule_deps.append(target)
    compilation_contexts = (
        _get_compilation_contexts_from_aspect_info_deps(
            rule_deps,
            aspect_info,
        ) +
        _get_compilation_contexts_from_deps(deps)
    )
    compilation_context, compilation_outputs = cc_common.compile(
        actions = ctx.actions,
        name = name,
        cc_toolchain = cc_toolchain,
        feature_configuration = feature_configuration,
        user_compile_flags = copts,
        system_includes = [cc_include_dir],
        srcs = srcs,
        public_hdrs = hdrs,
        compilation_contexts = compilation_contexts,
    )

    linking_contexts = (
        _get_linking_contexts_from_aspect_info_deps(
            rule_deps,
            aspect_info,
        ) +
        _get_linking_contexts_from_deps(deps)
    )
    linking_context, linking_outputs = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        name = name,
        compilation_outputs = compilation_outputs,
        cc_toolchain = cc_toolchain,
        feature_configuration = feature_configuration,
        linking_contexts = linking_contexts,
    )

    cc_info = CcInfo(
        compilation_context = compilation_context,
        linking_context = linking_context,
    )

    return struct(
        cc_info = cc_info,
        compilation_outputs = compilation_outputs,
        linking_outputs = linking_outputs,
    )

def _c_generator_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs
    adapter = target[IdlAdapterAspectInfo]

    interface_outputs, cc_include_dir = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._interface_generator,
        ctx.attr._interface_templates,
        _INTERFACE_GENERATOR_C_OUTPUT_MAPPING,
        visibility_control_template = ctx.file._interface_visibility_control_template,
        mnemonic = "Ros2IdlGeneratorC",
        progress_message = "Generating C IDL interfaces for %{label}",
    )

    typesupport_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._typesupport_generator,
        ctx.attr._typesupport_templates,
        _TYPESUPPORT_GENERATOR_C_OUTPUT_MAPPING,
        visibility_control_template = ctx.file._typesupport_introspection_visibility_control_template,
        extra_generator_args = [
            # TODO(mvukov) There are also rosidl_typesupport_connext_c and
            # rosidl_typesupport_fastrtps_c.
            "--typesupports=rosidl_typesupport_introspection_c",
        ],
        mnemonic = "Ros2IdlTypeSupportC",
        progress_message = "Generating C type support for %{label}",
    )

    typesupport_introspection_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._typesupport_introspection_generator,
        ctx.attr._typesupport_introspection_templates,
        _TYPESUPPORT_INTROSPECION_GENERATOR_C_OUTPUT_MAPPING,
        visibility_control_template = ctx.file._typesupport_introspection_visibility_control_template,
        mnemonic = "Ros2IdlTypeSupportIntrospectionC",
        progress_message = "Generating C type introspection support for %{label}",
    )

    all_outputs = interface_outputs + typesupport_outputs + typesupport_introspection_outputs
    hdrs = _get_hdrs(all_outputs)
    srcs = _get_srcs(all_outputs)

    compilation_info = _compile_cc_generated_code(
        ctx,
        name = package_name + "_c",
        aspect_info = CGeneratorAspectInfo,
        srcs = srcs,
        hdrs = hdrs,
        deps = ctx.attr._c_deps,
        cc_include_dir = cc_include_dir,
        copts = ctx.attr._c_copts,
    )

    return [
        CGeneratorAspectInfo(cc_info = compilation_info.cc_info),
    ]

c_generator_aspect = aspect(
    implementation = _c_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
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
        "_c_copts": attr.string_list(
            default = C_COPTS,
        ),
        "_c_deps": attr.label_list(
            default = [
                Label("@ros2_rosidl//:rosidl_runtime_c"),
                Label("@ros2_rosidl//:rosidl_typesupport_introspection_c"),
                Label("@ros2_rosidl_typesupport//:rosidl_typesupport_c"),
            ],
            providers = [CcInfo],
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    required_providers = [Ros2InterfaceInfo],
    required_aspect_providers = [IdlAdapterAspectInfo],
    provides = [CGeneratorAspectInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

def _cc_generator_impl(ctx, aspect_info):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [dep[aspect_info].cc_info for dep in ctx.attr.deps],
    )
    return [cc_info]

def _c_ros2_interface_library_impl(ctx):
    return _cc_generator_impl(ctx, CGeneratorAspectInfo)

c_ros2_interface_library = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [idl_adapter_aspect, c_generator_aspect],
            providers = [Ros2InterfaceInfo],
        ),
    },
    implementation = _c_ros2_interface_library_impl,
)

CppGeneratorAspectInfo = provider("TBD", fields = [
    "cc_info",
    "compilation_outputs",
])

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
    adapter = target[IdlAdapterAspectInfo]

    interface_outputs, cc_include_dir = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._interface_generator,
        ctx.attr._interface_templates,
        _INTERFACE_GENERATOR_CPP_OUTPUT_MAPPING,
        mnemonic = "Ros2IdlGeneratorCpp",
        progress_message = "Generating C++ IDL interfaces for %{label}",
    )

    typesupport_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._typesupport_generator,
        ctx.attr._typesupport_templates,
        _TYPESUPPORT_GENERATOR_CPP_OUTPUT_MAPPING,
        extra_generator_args = [
            # TODO(mvukov) There are also rosidl_typesupport_connext_cpp and
            # rosidl_typesupport_fastrtps_cpp.
            "--typesupports=rosidl_typesupport_introspection_cpp",
        ],
        mnemonic = "Ros2IdlTypeSupportCpp",
        progress_message = "Generating C++ type support for %{label}",
    )

    typesupport_introspection_outputs, _ = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._typesupport_introspection_generator,
        ctx.attr._typesupport_introspection_templates,
        _TYPESUPPORT_INTROSPECION_GENERATOR_CPP_OUTPUT_MAPPING,
        mnemonic = "Ros2IdlTypeSupportIntrospectionCpp",
        progress_message = "Generating C++ type introspection support for %{label}",
    )

    all_outputs = interface_outputs + typesupport_outputs + typesupport_introspection_outputs
    hdrs = _get_hdrs(all_outputs)
    srcs = _get_srcs(all_outputs)

    compilation_info = _compile_cc_generated_code(
        ctx,
        name = package_name + "_cpp",
        aspect_info = CppGeneratorAspectInfo,
        srcs = srcs,
        hdrs = hdrs,
        deps = ctx.attr._cpp_deps,
        cc_include_dir = cc_include_dir,
        copts = ctx.attr._cpp_copts,
    )

    return [
        CppGeneratorAspectInfo(
            cc_info = compilation_info.cc_info,
            compilation_outputs = compilation_info.compilation_outputs,
        ),
    ]

cpp_generator_aspect = aspect(
    implementation = _cpp_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
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
        "_cpp_copts": attr.string_list(
            default = CPP_COPTS,
        ),
        "_cpp_deps": attr.label_list(
            default = [
                Label("@ros2_rosidl//:rosidl_runtime_cpp"),
                Label("@ros2_rosidl//:rosidl_typesupport_introspection_c"),
                Label("@ros2_rosidl//:rosidl_typesupport_introspection_cpp"),
                Label("@ros2_rosidl_typesupport//:rosidl_typesupport_cpp"),
            ],
            providers = [CcInfo],
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    required_providers = [Ros2InterfaceInfo],
    required_aspect_providers = [IdlAdapterAspectInfo],
    provides = [CppGeneratorAspectInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

def _cpp_ros2_interface_library_impl(ctx):
    return _cc_generator_impl(ctx, CppGeneratorAspectInfo)

cpp_ros2_interface_library = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [idl_adapter_aspect, cpp_generator_aspect],
            providers = [Ros2InterfaceInfo],
        ),
    },
    implementation = _cpp_ros2_interface_library_impl,
)

PyGeneratorAspectInfo = provider("TBD", fields = [
    "cc_info",
    "dynamic_libraries",
    "transitive_sources",
    "imports",
    "linker_inputs",
])

_INTERFACE_GENERATOR_PY_OUTPUT_MAPPING = [
    "_%s.py",
    "_%s_s.c",
]

def _get_py_srcs(files):
    return [f for f in files if f.path.endswith(".py")]

def _py_generator_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs
    adapter = target[IdlAdapterAspectInfo]

    type_support_impl = "rosidl_typesupport_c"
    extra_generated_outputs = [
        "_{}_s.ep.{}.c".format(package_name, type_support_impl),
    ]

    for ext in ["action", "msg", "srv"]:
        if any([f.extension == ext for f in srcs]):
            extra_generated_outputs.append("{}/__init__.py".format(ext))

    interface_outputs, cc_include_dir = _run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._py_interface_generator,
        ctx.attr._py_interface_templates,
        _INTERFACE_GENERATOR_PY_OUTPUT_MAPPING,
        extra_generator_args = [
            "--typesupport-impls={}".format(type_support_impl),
        ],
        extra_generated_outputs = extra_generated_outputs,
        mnemonic = "Ros2IdlGeneratorPy",
        progress_message = "Generating Python IDL interfaces for %{label}",
    )

    all_outputs = interface_outputs

    cc_srcs = _get_srcs(all_outputs)
    py_extension_name = "{}_s__{}".format(package_name, type_support_impl)
    compilation_info = _compile_cc_generated_code(
        ctx,
        name = package_name + "_py",
        aspect_info = CGeneratorAspectInfo,
        srcs = cc_srcs,
        hdrs = [],
        deps = ctx.attr._py_ext_c_deps,
        cc_include_dir = cc_include_dir,
        copts = ctx.attr._py_ext_c_copts,
        target = target,
    )

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    # This one is needed because Python extensions are linked: if an IDL target
    # B depends on an IDL target A, then the Python extension of B depends
    # on the one of A.
    linking_contexts = _get_linking_contexts_from_aspect_info_deps(
        ctx.rule.attr.deps,
        PyGeneratorAspectInfo,
    )
    dynamic_library_name_stem = package_name + "/" + py_extension_name
    linking_outputs = cc_common.link(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        compilation_outputs = compilation_info.compilation_outputs,
        linking_contexts = [compilation_info.cc_info.linking_context] + linking_contexts,
        name = dynamic_library_name_stem,
        output_type = "dynamic_library",
        # TODO(mvukov) More deps means larger libs. Try to set this to False.
        # link_deps_statically = True,  # Default is True!
    )
    library_to_link = linking_outputs.library_to_link
    dynamic_library = ctx.actions.declare_file(
        dynamic_library_name_stem + ".so",
    )
    ctx.actions.symlink(
        output = dynamic_library,
        target_file = library_to_link.resolved_symlink_dynamic_library,
    )

    relative_path_parts = paths.relativize(
        cc_include_dir,
        ctx.bin_dir.path,
    ).split("/")
    if relative_path_parts[0] == "external":
        py_import_path = paths.join(*relative_path_parts[1:])
    else:
        py_import_path = paths.join(
            ctx.workspace_name,
            *relative_path_parts[0:]
        )

    py_info = PyGeneratorAspectInfo(
        cc_info = cc_common.merge_cc_infos(
            direct_cc_infos = [compilation_info.cc_info] + [
                dep[PyGeneratorAspectInfo].cc_info
                for dep in ctx.rule.attr.deps
            ],
        ),
        dynamic_libraries = depset(
            direct = [dynamic_library],
            transitive = [
                dep[PyGeneratorAspectInfo].dynamic_libraries
                for dep in ctx.rule.attr.deps
            ],
        ),
        transitive_sources = depset(
            direct = _get_py_srcs(all_outputs),
            transitive = [
                dep[PyGeneratorAspectInfo].transitive_sources
                for dep in ctx.rule.attr.deps
            ],
        ),
        imports = depset(
            direct = [py_import_path],
            transitive = [
                dep[PyGeneratorAspectInfo].imports
                for dep in ctx.rule.attr.deps
            ],
        ),
        linker_inputs = compilation_info.cc_info.linking_context.linker_inputs,
    )
    return [
        py_info,
    ]

py_generator_aspect = aspect(
    implementation = _py_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_py_interface_generator": attr.label(
            default = Label("@ros2_rosidl_python//:rosidl_generator_py_app"),
            executable = True,
            cfg = "exec",
        ),
        "_py_interface_templates": attr.label(
            default = Label("@ros2_rosidl_python//:rosidl_generator_py_templates"),
        ),
        "_py_ext_c_copts": attr.string_list(
            default = C_COPTS,
        ),
        "_py_ext_c_deps": attr.label_list(
            default = [
                Label("@rules_python//python/cc:current_py_cc_headers"),
                Label("@rules_ros2_pip_deps_numpy//:headers"),
            ],
            providers = [CcInfo],
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    required_providers = [Ros2InterfaceInfo],
    required_aspect_providers = [
        [IdlAdapterAspectInfo],
        [CGeneratorAspectInfo],
    ],
    provides = [PyGeneratorAspectInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

def _merge_py_generator_aspect_infos(py_infos):
    return PyGeneratorAspectInfo(
        dynamic_libraries = depset(
            transitive = [info.dynamic_libraries for info in py_infos],
        ),
        transitive_sources = depset(
            transitive = [info.transitive_sources for info in py_infos],
        ),
        imports = depset(transitive = [info.imports for info in py_infos]),
        linker_inputs = depset(transitive = [info.linker_inputs for info in py_infos]),
    )

def _py_generator_impl(ctx):
    py_info = _merge_py_generator_aspect_infos([
        dep[PyGeneratorAspectInfo]
        for dep in ctx.attr.deps
    ])

    linked_dynamic_libraries = []
    for linker_input in py_info.linker_inputs.to_list():
        for library in linker_input.libraries:
            if library.pic_static_library == None:
                # The sole purpose of this shenanigan is to fetch @ros2_rosidl//:rosidl_typesupport_introspection_c_identifier.
                # For that target we know it only has a dynamic library.
                linked_dynamic_libraries.append(library.dynamic_library)

    return [
        DefaultInfo(runfiles = ctx.runfiles(
            transitive_files = depset(
                transitive = [
                    py_info.transitive_sources,
                    py_info.dynamic_libraries,
                    depset(linked_dynamic_libraries),
                ],
            ),
        )),
        PyInfo(
            transitive_sources = py_info.transitive_sources,
            imports = py_info.imports,
        ),
    ]

py_generator = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [
                idl_adapter_aspect,
                c_generator_aspect,
                py_generator_aspect,
            ],
            providers = [Ros2InterfaceInfo],
        ),
    },
    implementation = _py_generator_impl,
)

def py_ros2_interface_library(name, deps, **kwargs):
    name_py = name + "_py_generator"
    py_generator(
        name = name_py,
        deps = deps,
        **kwargs
    )
    py_library(
        name = name,
        deps = [
            name_py,
            "@ros2_rosidl//:rosidl_parser",
            requirement("numpy"),
        ],
        **kwargs
    )
