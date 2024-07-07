# Copyright 2024 Milan Vukov
# Copyright 2024 otiv.ai
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
""" ROS 2 Rust IDL handling.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "CGeneratorAspectInfo",
    "IdlAdapterAspectInfo",
    "Ros2InterfaceInfo",
    "c_generator_aspect",
    "idl_adapter_aspect",
    "run_generator",
)
load("@rules_rust//rust:defs.bzl", "rust_common")
load("@rules_rust//rust/private:rustc.bzl", "rustc_compile_action")
load(
    "@rules_rust//rust/private:utils.bzl",
    "can_build_metadata",
    "compute_crate_name",
    "crate_root_src",
    "determine_lib_name",
    "determine_output_hash",
    "find_toolchain",
    "generate_output_diagnostics",
    "transform_deps",
    "transform_sources",
)

RustGeneratorAspectInfo = provider("TBD", fields = [
    "dep_variant_info",
    "transitive_dep_infos",
])

def _get_crate_info(providers):
    """Finds the CrateInfo provider in the list of providers."""
    for provider in providers:
        if hasattr(provider, "name"):
            return provider
    fail("Couldn't find a CrateInfo in the list of providers")

def _get_dep_info(providers):
    """Finds the DepInfo provider in the list of providers."""
    for provider in providers:
        if hasattr(provider, "direct_crates"):
            return provider
    fail("Couldn't find a DepInfo in the list of providers")

def _get_cc_info(providers):
    """Finds the CcInfo provider in the list of providers."""
    for provider in providers:
        if hasattr(provider, "linking_context"):
            return provider
    fail("Couldn't find a CcInfo in the list of providers")

def _compile_rust_code(ctx, label, crate_name, srcs, deps):
    toolchain = find_toolchain(ctx)

    crate_type = "rlib"
    crate_name = compute_crate_name(ctx.workspace_name, label, toolchain, crate_name)
    crate_root = crate_root_src(label, srcs, crate_type)
    srcs, crate_root = transform_sources(ctx, srcs, crate_root)

    output_hash = determine_output_hash(crate_root, label)
    rust_lib_name = determine_lib_name(
        crate_name,
        crate_type,
        toolchain,
        output_hash,
    )
    rust_lib = ctx.actions.declare_file(rust_lib_name)
    rust_metadata = None
    rustc_rmeta_output = None
    if can_build_metadata(toolchain, ctx, crate_type):
        rust_metadata = ctx.actions.declare_file(
            paths.replace_extension(rust_lib_name, ".rmeta"),
            sibling = rust_lib,
        )
        rustc_rmeta_output = generate_output_diagnostics(ctx, rust_metadata)

    providers = rustc_compile_action(
        ctx = ctx,
        attr = ctx.rule.attr,
        toolchain = toolchain,
        output_hash = output_hash,
        crate_info_dict = dict(
            name = crate_name,
            type = crate_type,
            root = crate_root,
            srcs = depset(srcs),
            deps = deps,
            proc_macro_deps = depset([]),
            aliases = {},
            output = rust_lib,
            rustc_output = generate_output_diagnostics(ctx, rust_lib),
            metadata = rust_metadata,
            rustc_rmeta_output = rustc_rmeta_output,
            edition = "2021",
            rustc_env = {},
            rustc_env_files = [],
            is_test = False,
            data = depset([]),
            compile_data = depset([]),
            compile_data_targets = depset([]),
            owner = label,
        ),
    )

    return rust_common.dep_variant_info(
        crate_info = _get_crate_info(providers),
        dep_info = _get_dep_info(providers),
        cc_info = _get_cc_info(providers),
    )

def _rust_generator_aspect_impl(target, ctx):
    package_name = target.label.name
    srcs = target[Ros2InterfaceInfo].info.srcs
    adapter = target[IdlAdapterAspectInfo]

    type_support_impl = "rosidl_typesupport_c"

    lib_rs = "rust/src/lib.rs"
    extra_generated_outputs = [lib_rs]
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

    # Ideally dep_variant_info could be a depset, and all dep propagation should
    # go via that depset. However, something in the generated dep_variant_info
    # from the compiled code is dynamic and Bazel complains. Therefore, I opted
    # to route dep info via transitive_dep_infos, like it's done in rust_prost_aspect.
    runtime_deps = transform_deps(ctx.attr._rust_deps)
    transitive_deps = []
    for dep in ctx.rule.attr.deps:
        dep_aspect_info = dep[RustGeneratorAspectInfo]
        transitive_deps.append(depset(
            [dep_aspect_info.dep_variant_info],
            transitive = [dep_aspect_info.transitive_dep_infos],
        ))

    dep_variant_info = _compile_rust_code(
        ctx,
        label = target.label,
        crate_name = package_name,
        srcs = interface_outputs,
        deps = depset(
            direct = runtime_deps,
            transitive = transitive_deps,
        ),
    )

    return RustGeneratorAspectInfo(
        dep_variant_info = dep_variant_info,
        transitive_dep_infos = depset(transitive = transitive_deps),
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
        "_rust_deps": attr.label_list(
            default = [
                "@ros2_rust//:rosidl_runtime_rs",
                "@rules_ros2_crate_index//:serde",
            ],
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
        "_process_wrapper": attr.label(
            default = Label("@rules_rust//util/process_wrapper"),
            executable = True,
            allow_single_file = True,
            cfg = "exec",
        ),
        "_rustc_output_diagnostics": attr.label(
            default = Label("@rules_rust//:rustc_output_diagnostics"),
        ),
    },
    required_providers = [Ros2InterfaceInfo],
    required_aspect_providers = [
        [IdlAdapterAspectInfo],
        [CGeneratorAspectInfo],
    ],
    provides = [RustGeneratorAspectInfo],
    toolchains = [
        "@bazel_tools//tools/cpp:toolchain_type",
        "@rules_rust//rust:toolchain_type",
    ],
    fragments = ["cpp"],
)

def _rust_ros2_interface_library_impl(ctx):
    dep_variant_cc_info = rust_common.dep_variant_info(
        cc_info = cc_common.merge_cc_infos(
            direct_cc_infos = [dep[CGeneratorAspectInfo].cc_info for dep in ctx.attr.deps],
        ),
        crate_info = None,
        dep_info = None,
    )
    return [
        DefaultInfo(
            files = depset([
                dep[RustGeneratorAspectInfo].dep_variant_info.crate_info.output
                for dep in ctx.attr.deps
            ]),
        ),
        rust_common.crate_group_info(
            dep_variant_infos = depset(
                [
                    dep[RustGeneratorAspectInfo].dep_variant_info
                    for dep in ctx.attr.deps
                ] + [dep_variant_cc_info],
            ),
        ),
    ]

rust_ros2_interface_library = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [
                idl_adapter_aspect,
                c_generator_aspect,
                rust_generator_aspect,
            ],
            providers = [Ros2InterfaceInfo],
        ),
    },
    implementation = _rust_ros2_interface_library_impl,
)
