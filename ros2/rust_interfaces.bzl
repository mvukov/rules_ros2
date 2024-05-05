# Copyright 2024 Milan Vukov
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

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "IdlAdapterAspectInfo",
    "Ros2InterfaceInfo",
    "idl_adapter_aspect",
    "run_generator",
)

RustGeneratorAspectInfo = provider("TBD", fields = [
    "files",
])

def _rust_generator_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs
    adapter = target[IdlAdapterAspectInfo]

    type_support_impl = "rosidl_typesupport_c"

    extra_generated_outputs = ["rust/src/lib.rs"]
    for ext in ["msg", "srv"]:
        if any([f.extension == ext for f in srcs]):
            extra_generated_outputs.append("rust/src/{}.rs".format(ext))

    interface_outputs, _ = run_generator(
        ctx,
        srcs,
        package_name,
        adapter,
        ctx.executable._rust_interface_generator,
        ctx.attr._rust_interface_templates,
        [],
        extra_generator_args = [
            "--typesupport-impls={}".format(type_support_impl),
        ],
        extra_generated_outputs = extra_generated_outputs,
        mnemonic = "Ros2IdlGeneratorRust",
        progress_message = "Generating Rust IDL interfaces for %{label}",
    )

    return RustGeneratorAspectInfo(
        files = depset(
            interface_outputs,
            transitive = [dep[RustGeneratorAspectInfo].files for dep in ctx.rule.attr.deps],
        ),
    )

rust_generator_aspect = aspect(
    implementation = _rust_generator_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_rust_interface_generator": attr.label(
            default = Label("@ros2_rust//:rosidl_generator_rs_app"),
            executable = True,
            cfg = "exec",
        ),
        "_rust_interface_templates": attr.label(
            default = Label("@ros2_rust//:rosidl_generator_rs_templates"),
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    required_providers = [Ros2InterfaceInfo],
    required_aspect_providers = [IdlAdapterAspectInfo],
    provides = [RustGeneratorAspectInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

def _rust_generator_impl(ctx):
    files = depset(transitive = [dep[RustGeneratorAspectInfo].files for dep in ctx.attr.deps])
    return DefaultInfo(files = files)

rust_generator = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [
                idl_adapter_aspect,
                rust_generator_aspect,
            ],
            providers = [DefaultInfo],
        ),
    },
    implementation = _rust_generator_impl,
)
