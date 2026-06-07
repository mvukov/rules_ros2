# Plan: Sized arrays via `ros2_field_options.proto`

## Context

`proto_to_ros2_msg.py` always emits dynamic arrays (`T[]`) for `bytes` fields and
`repeated T` fields. Users need a way to annotate proto fields to emit **fixed-size
arrays** (`T[N]`) in ROS 2 msg format. Fixed-size arrays avoid heap allocation in
the ROS 2 serialization layer and are important for performance-critical types
(hashes, fixed-length point clouds, pose arrays, etc.).

The mechanism: a new custom proto field option `(ros2.msg.array_size) = N` defined in
`ros2/protobuf/ros2_field_options.proto`. protoc serialises this as extension field
bytes inside `FieldOptions` in the descriptor set. The Python tool reads it via
`field.options.UnknownFields()` — no `py_proto_library` Bazel target is needed.

---

## New file: `ros2/protobuf/ros2_field_options.proto`

```protobuf
syntax = "proto3";
package ros2.msg;

import "google/protobuf/descriptor.proto";

extend google.protobuf.FieldOptions {
  // Fixed array size for the generated ROS .msg field.
  // Valid on: singular bytes fields and repeated fields of any type.
  // When N > 0 the generator emits T[N] instead of T[].
  // Value 0 (the proto3 default) means dynamic array (no change).
  uint32 array_size = 50000;
}
```

Field number 50000 is in the open custom-extension range.

---

## Changes

### 1. `ros2/protobuf/BUILD.bazel`

Add a `proto_library` for the new options file (public so downstream `proto_library`
targets can depend on it):

```python
proto_library(
    name = "ros2_field_options_proto",
    srcs = ["ros2_field_options.proto"],
    deps = ["@com_google_protobuf//:descriptor_proto"],
    visibility = ["//visibility:public"],
)
```

No changes to the `proto_to_ros2_msg` `py_binary` deps — the tool reads the option
via `UnknownFields()` at runtime without importing a generated Python extension module.

### 2. `ros2/protobuf/proto_to_ros2_msg.py`

**Add constant and helper** near the top of the file (after imports):

```python
# Field number for the ros2.msg.array_size FieldOptions extension.
_ARRAY_SIZE_FIELD_NUMBER = 50000


def _get_array_size(field_options):
    """Returns the (ros2.msg.array_size) value from field options, or 0."""
    for uf in field_options.UnknownFields():
        if uf.field_number == _ARRAY_SIZE_FIELD_NUMBER:
            return uf.data  # wire type 0 (varint) → Python int
    return 0
```

**Modify the field-emission loop** in `_convert()`:

After the `deprecated` guard, read `array_size` once per field and validate:

```python
array_size = _get_array_size(field.options)
is_repeated = (field.label == FieldDescriptorProto.LABEL_REPEATED)

if array_size and not is_repeated and field_type_value != FieldDescriptorProto.TYPE_BYTES:
    sys.exit(
        f'Error: {proto_source}: field "{field.name}" has array_size={array_size} '
        f'but is not a bytes field or a repeated field.')
```

Then change each array-suffix site (three places):

| Field kind                      | Before                                  | After                                                                   |
| ------------------------------- | --------------------------------------- | ----------------------------------------------------------------------- |
| `TYPE_MESSAGE` (repeated)       | `ros2_type + '[]'`                      | `ros2_type + f'[{array_size}]'` if `array_size` else `ros2_type + '[]'` |
| `TYPE_ENUM`                     | `'int32[]' if is_repeated else 'int32'` | `f'int32[{array_size}]'` if `array_size and is_repeated` else original  |
| scalar / bytes (bottom of loop) | `ros2_type + '[]'`                      | sized suffix for repeated and for singular bytes                        |

Concretely, the last block becomes:

```python
ros2_type = _PROTO_TO_ROS_TYPE[field_type_value]

if is_repeated and field_type_value == FieldDescriptorProto.TYPE_BYTES:
    sys.exit(...)  # unchanged

suffix = f'[{array_size}]' if array_size else '[]'

if field_type_value == FieldDescriptorProto.TYPE_BYTES:
    # bytes already maps to 'uint8[]'; replace the trailing [] with the suffix.
    ros2_type = 'uint8' + suffix
elif is_repeated:
    ros2_type = ros2_type + suffix
# singular non-bytes: ros2_type unchanged (no suffix)

lines.append(f'{ros2_type} {field.name}')
```

### 3. Test fixtures — `ros2/test/protobuf/` (new files)

**`sized_bytes.proto`** — singular `bytes` with and without a size:

```protobuf
syntax = "proto3";
package ros2.test.protobuf;
import "ros2/protobuf/ros2_field_options.proto";
message SizedBytes {
  bytes hash = 1 [(ros2.msg.array_size) = 32];
  bytes raw = 2;
}
```

Expected: `uint8[32] hash`, `uint8[] raw`.

**`sized_array.proto`** — repeated scalar and repeated message with fixed sizes:

```protobuf
syntax = "proto3";
package ros2.test.protobuf;
import "ros2/protobuf/ros2_field_options.proto";
import "ros2/test/protobuf/point.proto";
message SizedArray {
  repeated float coords = 1 [(ros2.msg.array_size) = 3];
  repeated float dynamic = 2;
  repeated ros2.test.protobuf.Point pts = 3 [(ros2.msg.array_size) = 4];
}
```

Expected: `float32[3] coords`, `float32[] dynamic`,
`ros2_test_protobuf_point_proto_ros_msgs/Point[4] pts`.

**`sized_bad.proto`** — singular non-bytes scalar with `array_size` (error test):

```protobuf
syntax = "proto3";
package ros2.test.protobuf;
import "ros2/protobuf/ros2_field_options.proto";
message SizedBad {
  int32 x = 1 [(ros2.msg.array_size) = 3];
}
```

### 4. `ros2/test/protobuf/BUILD.bazel`

Add three `proto_library` targets (the `sized_array_proto` depends on `:point_proto`):

```python
proto_library(
    name = "sized_bytes_proto",
    srcs = ["sized_bytes.proto"],
    deps = ["//ros2/protobuf:ros2_field_options_proto"],
)

proto_library(
    name = "sized_array_proto",
    srcs = ["sized_array.proto"],
    deps = [
        ":point_proto",
        "//ros2/protobuf:ros2_field_options_proto",
    ],
)

proto_library(
    name = "sized_bad_proto",
    srcs = ["sized_bad.proto"],
    deps = ["//ros2/protobuf:ros2_field_options_proto"],
)
```

Add all three to the `data` list of `proto_to_ros2_msg_tests`.

### 5. `ros2/test/protobuf/tests.py`

Add a new class **`SizedArrayTests`**:

```python
class SizedArrayTests:
    def test_bytes_with_array_size(self, tmp_path):
        ...  # assert 'uint8[32] hash' and 'uint8[] raw' in output

    def test_repeated_scalar_with_array_size(self, tmp_path):
        ...  # assert 'float32[3] coords' and 'float32[] dynamic' in output

    def test_repeated_message_with_array_size(self, tmp_path):
        ...  # assert '<pkg>/Point[4] pts' in output

    def test_array_size_on_singular_scalar_is_error(self, tmp_path):
        ...  # assert SystemExit with 'array_size' in message
```

The `test_repeated_message_with_array_size` test must supply the msg_type_map for
`.ros2.test.protobuf.Point` (same pattern as `test_message_dep_field`).

### 6. `ros2/protobuf/README.md`

Add a **Fixed-size arrays** entry to the Limitations section and a prose paragraph
documenting the option and import path.

---

## Out of scope

`proto_to_ros2_converter.py` is not changed here. Fixed-size ROS arrays require
different C++ copy logic (`std::copy` / `memcpy` vs. `assign`); that is a follow-up.

---

## Verification

```
bazel test //ros2/test/protobuf:proto_to_ros2_msg_tests --test_output=streamed
```

All tests in `SizedArrayTests` must pass; all pre-existing tests must continue to pass.
