# Examples for rules_ros2

This folder provides some examples and guidelines on how you can use this repo
in your very own (mono)repo.

## Handling of external dependencies

It's important to note here that this folder is
a yet another [repository](https://bazel.build/concepts/build-ref#repositories).
External dependencies are handled with
[bzlmod](https://bazel.build/external/overview#bzlmod). Please take a look at the
`MODULE.bazel` file in this folder to get an idea how to set up yourself.

This folder imports `rules_ros2` with a `local_path_override`. In a normal workflow,
you would typically import `rules_ros2` into your (mono)repo as follows:

```starlark
bazel_dep(name = "com_github_mvukov_rules_ros2")
archive_override(
    module_name = "com_github_mvukov_rules_ros2",
    # Here you can use e.g. `openssl dgst -sha256 -binary <archive> | base64` to compute the hash.
    integrity = "sha256-<base64-encoded sha256 of the .tar.gz archive below>",
    strip_prefix = "rules_ros2-<git commit SHA>",
    urls = ["https://github.com/mvukov/rules_ros2/archive/<git commit SHA>.tar.gz"],
)
```

`rules_ros2` defines a Python 3.12 interpreter and defines the hub with resolved Python deps. If you want a different intepreter and/or you want to extend the Python requirements files with extra deps you might want to use in your monorepo, you'll need to patch `requirements_lock.txt` file and this repo.

## The rest of the setup

Besides setting up your `MODULE.bazel` file, please make sure you copy the `.bazelrc`
file as well to your monorepo -- or at least adjust your own using the provided
one.

Before you build/run/test any of the targets in this folder, please make sure
your terminal is in this folder, or in a subfolder, and not in the root
of the repo.

You can start with a simple [chatter](chatter) example.
