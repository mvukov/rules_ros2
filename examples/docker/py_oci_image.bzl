# Adapted from https://raw.githubusercontent.com/aspect-build/bazel-examples/b7d38d45c97eadac27a2c5be7e288abbb49db78d/oci_python_image/py_layer.bzl
"Wrapper macro to make three separate layers for python applications"

load("@aspect_bazel_lib//lib:tar.bzl", "mtree_spec", "tar")
load(
    "@com_github_mvukov_rules_ros2//third_party:expand_template.bzl",
    "expand_template",
)
load("@rules_oci//oci:defs.bzl", "oci_image")

# match *only* external repositories that have the string "python"
# e.g. this will match
#   `/hello_world/hello_world_bin.runfiles/rules_python~0.21.0~python~python3_9_aarch64-unknown-linux-gnu/bin/python3`
# but not match
#   `/hello_world/hello_world_bin.runfiles/_main/python_app`
PY_INTERPRETER_REGEX = "\\.runfiles/.*python.*-.*"

# match *only* external pip like repositories that contain the string "site-packages"
SITE_PACKAGES_REGEX = "\\.runfiles/.*/site-packages/.*"

def py_layers(name, binary):
    """Create three layers for a py_binary target: interpreter, third-party dependencies, and application code.

    This allows a container image to have smaller uploads, since the application layer usually changes more
    than the other two.

    Args:
        name: prefix for generated targets, to ensure they are unique within the package
        binary: a py_binary target
    Returns:
        a list of labels for the layers, which are tar files
    """

    # Produce layers in this order, as the app changes most often
    layers = ["interpreter", "packages", "app"]

    # Produce the manifest for a tar file of our py_binary, but don't tar it up yet, so we can split
    # into fine-grained layers for better docker performance.
    mtree_spec(
        name = name + ".mf",
        srcs = [binary],
    )

    native.genrule(
        name = name + ".interpreter_tar_manifest",
        srcs = [name + ".mf"],
        outs = [name + ".interpreter_tar_manifest.spec"],
        cmd = "grep -v '{}' $< | grep '{}' >$@".format(SITE_PACKAGES_REGEX, PY_INTERPRETER_REGEX),
    )

    native.genrule(
        name = name + ".packages_tar_manifest",
        srcs = [name + ".mf"],
        outs = [name + ".packages_tar_manifest.spec"],
        cmd = "grep '{}' $< >$@".format(SITE_PACKAGES_REGEX),
    )

    # Any lines that didn't match one of the two grep above
    native.genrule(
        name = name + ".app_tar_manifest",
        srcs = [name + ".mf"],
        outs = [name + ".app_tar_manifest.spec"],
        cmd = "grep -v '{}' $< | grep -v '{}' >$@".format(SITE_PACKAGES_REGEX, PY_INTERPRETER_REGEX),
    )

    result = []
    for layer in layers:
        layer_target = "{}.{}_layer".format(name, layer)
        result.append(layer_target)
        tar(
            name = layer_target,
            srcs = [binary],
            mtree = "{}.{}_tar_manifest".format(name, layer),
        )

    return result

def py_oci_image(name, binary, tars = [], **kwargs):
    "Wrapper around oci_image that splits the py_binary into layers."

    entrypoint = name + ".entrypoint"
    expand_template(
        name = entrypoint,
        out = name + "_entrypoint.sh",
        data = [binary],
        substitutions = {
            "{{binary_path}}": "$(rootpath {})".format(binary),
        },
        template = "entrypoint.sh.tpl",
        is_executable = True,
        tags = ["manual"],
    )

    entrypoint_tar = entrypoint + ".tar"
    tar(
        name = entrypoint_tar,
        srcs = [entrypoint],
        mtree = [
            "entrypoint.sh uid=0 gid=0 time=0 mode=0755 type=file content=$(execpath {})".format(entrypoint),
        ],
        tags = ["manual"],
    )

    oci_image(
        name = name,
        entrypoint = ["/entrypoint.sh"],
        tars = tars + py_layers(name, binary) + [entrypoint_tar],
        **kwargs
    )
