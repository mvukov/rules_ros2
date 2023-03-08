# Copyright (c) 2021 Digital Asset (Switzerland) GmbH and/or its affiliates. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# Adapted from https://github.com/digital-asset/daml/pull/8428

def _symlink_impl(ctx):
    executable = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(
        output = executable,
        target_file = ctx.executable.executable,
        is_executable = True,
    )

    runfiles = ctx.runfiles(files = [ctx.executable.executable])
    runfiles = runfiles.merge(ctx.attr.executable[DefaultInfo].default_runfiles)
    return [DefaultInfo(
        executable = executable,
        files = depset(direct = [executable]),
        runfiles = runfiles,
    )]

symlink = rule(
    _symlink_impl,
    attrs = {
        "executable": attr.label(
            executable = True,
            cfg = "target",
        ),
    },
    executable = True,
    doc = "Creates a new target for the given executable.",
)
