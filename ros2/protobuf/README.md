# Proto → ROS 2 interface rules

This package provides Bazel rules for converting [Protocol Buffer](https://protobuf.dev/)
message definitions into ROS 2 interface types and optional C++ conversion
helpers.

## Rules

### `proto_ros2_interface_library`

Generates one ROS 2 `.msg` file for each proto source in a single
`proto_library` dep and exposes `Ros2InterfaceInfo`. Use this rule when you
need the generated `.msg` files as input to other interface rules
(`c_ros2_interface_library`, `py_ros2_interface_library`, etc.) without
triggering a full C++ compilation.

```python
load("//ros2/protobuf:defs.bzl", "proto_ros2_interface_library")

proto_ros2_interface_library(
    name = "point_ros_msgs",
    dep = ":point_proto",
)
```

### `cpp_proto_ros2_interface_library`

Runs the full pipeline (proto → `.msg` → IDL → C++ headers + type-support
sources) and produces a `CcInfo` library. Use this rule when you need to
`#include` the generated message headers in C++ code.

```python
load("//ros2/protobuf:defs.bzl", "cpp_proto_ros2_interface_library")

cpp_proto_ros2_interface_library(
    name = "point_cpp_ros_msgs",
    deps = [":point_proto"],
)
```

Include path: `<ros_package_name>/msg/<MessageName>.hpp`
(e.g. `point_proto_ros_msgs/msg/point.hpp`).

### `cpp_proto_ros2_converter_library`

Generates a C++ library with bidirectional conversion functions between the
proto C++ type and the ROS 2 message type. The generated functions live in
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

| Proto source       | Required message name | Generated ROS package  |
| ------------------ | --------------------- | ---------------------- |
| `point.proto`      | `Point`               | `point_proto_ros_msgs` |
| `my_message.proto` | `MyMessage`           | `my_proto_ros_msgs`    |

The message name must be the PascalCase form of the filename stem. The ROS
package name is derived from the `proto_library` target name with `_ros_msgs`
appended.

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
- **Enums.** Only enums defined in the same proto file are supported.
  Cross-file enum references are not supported.
- **`oneof` fields** are not supported.
- **`map` fields** are not supported.
- **Group fields** are not supported.
- **`repeated bytes`** is not supported (`bytes` already maps to `uint8[]`;
  `repeated bytes` would require `uint8[][]`, which is not a valid ROS 2 type).
