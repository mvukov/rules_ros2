# Proto → ROS 2 interface rules

This package provides Bazel rules for converting [Protocol Buffer](https://protobuf.dev/)
message definitions into ROS 2 interface types and optional C++ conversion
helpers.

## Rules

### `proto_ros2_interface_library`

Generates one ROS 2 `.msg` file for each proto source in a single
`proto_library` dep. Use this rule when you
need the generated `.msg` files as input to other interface rules
(`cpp_ros2_interface_library`, `py_ros2_interface_library`, etc.).

```python
load("//ros2/protobuf:defs.bzl", "proto_ros2_interface_library")

proto_ros2_interface_library(
    name = "point_msgs",
    dep = ":point_proto",
)
```

### `cpp_proto_ros2_interface_library`

Runs the full pipeline (proto → `.msg` → ... → C++ code → compilation). Use this rule when you need to
include the generated message headers in your C++ code.

```python
load("//ros2/protobuf:defs.bzl", "cpp_proto_ros2_interface_library")

cpp_proto_ros2_interface_library(
    name = "cpp_point_msgs",
    deps = [":point_proto"],
)
```

Include path: `<ros_package_name>/msg/<MessageName>.hpp`
(e.g. `src_foo_point_proto_ros_msgs/msg/Point.hpp` for `//src/foo:point_proto`).

### `cpp_proto_ros2_converter_library`

Generates C++ libraries with bidirectional conversion functions between the
proto C++ types and the corresponding ROS 2 message type. The generated functions live in
namespace `<ros_package_name>::proto_converters`:

```cpp
void ToRos(const MyProtoType& proto, MyRosType* ros);
void ToProto(const MyRosType& ros, MyProtoType* proto);
```

```python
load("//ros2/protobuf:defs.bzl", "cpp_proto_ros2_converter_library")

cpp_proto_ros2_converter_library(
    name = "point_converters",
    deps = [":point_proto"],
)
```

## Naming conventions

The message name must be the PascalCase form of the filename stem.

The ROS package name is derived from the effective proto import path of the
first source file (respecting `strip_import_prefix`) and the target name:

```
<import_dir>_<target_name>_ros_msgs
```

where `<import_dir>` is the directory part of the proto file's import path
after `strip_import_prefix` is applied, with `/` and `-` replaced by `_`.
When the import path has no directory component the prefix is omitted.

For ordinary local targets (no `strip_import_prefix`) the import path
equals the Bazel package path, so the result is equivalent to:

```
<bazel_package_path>_<target_name>_ros_msgs
```

| Label                                    | `strip_import_prefix` | Generated ROS package                      |
| ---------------------------------------- | --------------------- | ------------------------------------------ |
| `//src/foo:perf_proto`                   | _(none)_              | `src_foo_perf_proto_ros_msgs`              |
| `//src/bar:my_proto`                     | _(none)_              | `src_bar_my_proto_ros_msgs`                |
| `//:root_proto`                          | _(none)_              | `root_proto_ros_msgs`                      |
| `@com_google_protobuf//:timestamp_proto` | `/src`                | `google_protobuf_timestamp_proto_ros_msgs` |

Targets at the repository root with no `strip_import_prefix` (empty import
directory) have no prefix.

Rationale: In a monorepo, multiple Bazel packages can independently define a
`proto_library` with the same short target name (e.g. `//src/foo:perf_proto`
and `//src/bar:perf_proto`). Using only the target name for the ROS package
would produce the same string for both, causing output file collisions when
both are consumed by the same build target. Basing the name on the import path
rather than the raw Bazel package path also ensures that `strip_import_prefix`
is respected: e.g. Google's well-known protos strip their `src/` prefix, so
the generated name starts with `google_protobuf_` rather than
`src_google_protobuf_`. This makes the ROS package name match the proto import
namespace that users actually write in their `.proto` files.

## Type mapping

| Proto type                    | ROS 2 type                     |
| ----------------------------- | ------------------------------ |
| `double`                      | `float64`                      |
| `float`                       | `float32`                      |
| `int32`, `sint32`, `sfixed32` | `int32`                        |
| `int64`, `sint64`, `sfixed64` | `int64`                        |
| `uint32`, `fixed32`           | `uint32`                       |
| `uint64`, `fixed64`           | `uint64`                       |
| `bool`                        | `bool`                         |
| `string`                      | `string`                       |
| `bytes`                       | `uint8[]`                      |
| `enum`                        | `int32` (with named constants) |
| `message`                     | `<ros_package>/MsgName`        |
| `repeated T`                  | `T[]`                          |

## Limitations and constraints

- **One message per file.** Each `.proto` source must define exactly one
  top-level message, and its name must be the PascalCase form of the filename
  stem (e.g. `point.proto` → `Point`).
- **No services.** Service definitions in a proto file cause a build error.
- **Nested message types** are not supported. All message types used as fields
  must be defined at the top level of their own proto file.
- **Enums.** Only enums defined **inside the message** (nested enum types) are
  supported. File-level enum definitions cause a build error. Cross-file enum
  references are not supported.
- **`oneof` fields** are not supported.
- **`map` fields** are not supported.
- **Group fields** are not supported.
- **`repeated bytes`** is not supported (`bytes` already maps to `uint8[]`;
  `repeated bytes` would require `uint8[][]`, which is not a valid ROS 2 type).
