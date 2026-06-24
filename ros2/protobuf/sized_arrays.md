# Plan: Sized arrays as sized containers via `ros2_field_options.proto`

## Context

`proto_to_ros2_msg.py` always emits dynamic arrays (`T[]`) for `bytes` fields and
`repeated T` fields. Users need a way to annotate proto fields to emit **fixed-size
arrays** (`T[N]`) in ROS 2 msg format. Fixed-size arrays avoid heap allocation in
the ROS 2 serialization layer and are important for performance-critical types
(hashes, fixed-length point clouds, pose arrays, etc.).

### Sized-container model

A sized array `T[N]` is treated as a **capacity-bounded container**, not an
exact-count buffer. The actual number of valid elements may be less than `N`.
For each annotated field `<name>` the generator therefore emits:

1. `uint32 <UPPER_NAME>_MAX_SIZE=N` — a named constant declaring the capacity.
2. `T[N] <name>` — the fixed-size storage buffer.
3. `uint32 <name>_actual_size` — the number of valid elements in the buffer.

The converter uses `<name>_actual_size` in **both directions**:
- `ToRos`: sets `<name>_actual_size` from the proto field's element count
  (clamped to `N`).
- `ToProto`: slices the fixed buffer to `<name>_actual_size` elements before
  assigning to the proto field.

The mechanism: a new custom proto field option `(ros2.msg.array_size) = N` defined
in `ros2/protobuf/ros2_field_options.proto`. protoc serialises this as extension
field bytes inside `FieldOptions` in the descriptor set. The Python tool reads it
via `field.options.UnknownFields()` — no `py_proto_library` Bazel target is needed.

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

Add a `proto_library` for the new options file (public so downstream
`proto_library` targets can depend on it):

```python
proto_library(
    name = "ros2_field_options_proto",
    srcs = ["ros2_field_options.proto"],
    deps = ["@com_google_protobuf//:descriptor_proto"],
    visibility = ["//visibility:public"],
)
```

No changes to the `proto_to_ros2_msg` or `proto_to_ros2_converter` `py_binary`
deps — both tools read the option via `UnknownFields()` at runtime.

### 2. `ros2/protobuf/proto_to_ros2.py`

Add a shared constant and helper (used by both generator tools):

```python
# Field number for the ros2.msg.array_size FieldOptions extension.
ARRAY_SIZE_FIELD_NUMBER = 50000


def get_array_size(field_options):
    """Returns the (ros2.msg.array_size) value from field options, or 0."""
    for uf in field_options.UnknownFields():
        if uf.field_number == ARRAY_SIZE_FIELD_NUMBER:
            return uf.data  # wire type 0 (varint) → Python int
    return 0
```

### 3. `ros2/protobuf/proto_to_ros2_msg.py`

**Modify the field-emission loop** in `_convert()`.

After the `deprecated` guard, read `array_size` once per field and validate:

```python
array_size = proto_to_ros2.get_array_size(field.options)
is_repeated = (field.label == FieldDescriptorProto.LABEL_REPEATED)

if array_size and not is_repeated and field_type_value != FieldDescriptorProto.TYPE_BYTES:
    sys.exit(
        f'Error: {proto_source}: field "{field.name}" has array_size={array_size} '
        f'but is not a bytes field or a repeated field.')
```

For every field where `array_size > 0`, emit three items instead of one:

```
uint32 <UPPER_NAME>_MAX_SIZE=<N>
T[N] <name>
uint32 <name>_actual_size
```

Concretely, the array-suffix sites become:

| Field kind                | Before                      | After (array_size > 0)                              |
| ------------------------- | --------------------------- | --------------------------------------------------- |
| `TYPE_MESSAGE` (repeated) | `ros2_type + '[]'`          | `ros2_type + f'[{array_size}]'`                     |
| `TYPE_ENUM` (repeated)    | `'int32[]'`                 | `f'int32[{array_size}]'`                            |
| scalar `repeated`         | `ros2_type + '[]'`          | `ros2_type + f'[{array_size}]'`                     |
| `TYPE_BYTES` (singular)   | `'uint8[]'`                 | `f'uint8[{array_size}]'`                            |

Note: enum value constants (`KIND_NONE=0`, etc.) are emitted in the first pass
before the field loop; the `MAX_SIZE` constant is emitted inline per-field in the
field loop. There is no ordering conflict between the two.

Helper for emitting a sized field block:

```python
def _emit_sized_field(lines, upper_name, array_size, ros2_type, field_name):
    lines.append(f'uint32 {upper_name}_MAX_SIZE={array_size}')
    lines.append(f'{ros2_type} {field_name}')
    lines.append(f'uint32 {field_name}_actual_size')
```

Fields without `array_size` (i.e., `array_size == 0`) continue to emit a single
line as today.

### 4. `ros2/protobuf/proto_to_ros2_converter.py`

`_field_conversions` must read `array_size` and generate sized-container code
for annotated fields. The function signature is unchanged; it already receives
full `field` objects with `field.options`.

In every snippet below, `n` is computed with `std::min` in **both** directions:
- `ToRos` clamps the proto element count to the buffer capacity.
- `ToProto` clamps `_actual_size` to the buffer capacity so a corrupt ROS field
  cannot cause an out-of-bounds read.

**`bytes` (singular), `array_size > 0`:**

```cpp
// ToRos
const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.{name}().size()), {array_size}U);
ros->{name}_actual_size = n;
std::copy_n(proto.{name}().begin(), n, ros->{name}.begin());

// ToProto
const std::uint32_t n = std::min(ros.{name}_actual_size, {array_size}U);
proto->set_{name}(reinterpret_cast<const char*>(ros.{name}.data()), n);
```

**Scalar `repeated`, `array_size > 0`** (ROS type is `std::array<T, N>`):

```cpp
// ToRos
const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.{name}().size()), {array_size}U);
ros->{name}_actual_size = n;
std::copy_n(proto.{name}().begin(), n, ros->{name}.begin());

// ToProto
const std::uint32_t n = std::min(ros.{name}_actual_size, {array_size}U);
proto->mutable_{name}()->Assign(ros.{name}.begin(), ros.{name}.begin() + n);
```

**`repeated` message, `array_size > 0`** (ROS type is `std::array<MsgT, N>`):

```cpp
// ToRos
const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.{name}().size()), {array_size}U);
ros->{name}_actual_size = n;
for (std::uint32_t i = 0; i < n; ++i) {{
  {conv}::ToRos(proto.{name}(i), &ros->{name}[i]);
}}

// ToProto
const std::uint32_t n = std::min(ros.{name}_actual_size, {array_size}U);
proto->clear_{name}();
proto->mutable_{name}()->Reserve(n);
for (std::uint32_t i = 0; i < n; ++i) {{
  {conv}::ToProto(ros.{name}[i], proto->add_{name}());
}}
```

**`repeated` enum, `array_size > 0`** (ROS type is `std::array<int32_t, N>`;
the enum-value cast pattern mirrors the existing repeated-enum code):

```cpp
// ToRos
const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.{name}().size()), {array_size}U);
ros->{name}_actual_size = n;
for (std::uint32_t i = 0; i < n; ++i) {{
  ros->{name}[i] = static_cast<int32_t>(proto.{name}(i));
}}

// ToProto
const std::uint32_t n = std::min(ros.{name}_actual_size, {array_size}U);
proto->clear_{name}();
proto->mutable_{name}()->Reserve(n);
for (std::uint32_t i = 0; i < n; ++i) {{
  proto->add_{name}(static_cast<{enum_cpp_type}>(ros.{name}[i]));
}}
```

### 5. Test fixtures — `ros2/test/protobuf/` (new files)

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

Expected `.msg`:
```
uint32 HASH_MAX_SIZE=32
uint8[32] hash
uint32 hash_actual_size
uint8[] raw
```

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

Expected `.msg`:
```
uint32 COORDS_MAX_SIZE=3
float32[3] coords
uint32 coords_actual_size
float32[] dynamic
uint32 PTS_MAX_SIZE=4
ros2_test_protobuf_point_proto_ros_msgs/Point[4] pts
uint32 pts_actual_size
```

**`sized_enum.proto`** — repeated enum with a fixed size (inline enum; cross-file
enums are unsupported):

```protobuf
syntax = "proto3";
package ros2.test.protobuf;
import "ros2/protobuf/ros2_field_options.proto";
message SizedEnum {
  enum Kind {
    KIND_NONE = 0;
    KIND_A = 1;
  }
  repeated Kind kinds = 1 [(ros2.msg.array_size) = 4];
}
```

Expected `.msg`:
```
# Kind constants
int32 KIND_NONE=0
int32 KIND_A=1

uint32 KINDS_MAX_SIZE=4
int32[4] kinds
uint32 kinds_actual_size
```

**`sized_bad.proto`** — singular non-bytes scalar with `array_size` (error test):

```protobuf
syntax = "proto3";
package ros2.test.protobuf;
import "ros2/protobuf/ros2_field_options.proto";
message SizedBad {
  int32 x = 1 [(ros2.msg.array_size) = 3];
}
```

### 6. `ros2/test/protobuf/BUILD.bazel`

Add three `proto_library` targets (the `sized_array_proto` depends on
`:point_proto`):

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
    name = "sized_enum_proto",
    srcs = ["sized_enum.proto"],
    deps = ["//ros2/protobuf:ros2_field_options_proto"],
)

proto_library(
    name = "sized_bad_proto",
    srcs = ["sized_bad.proto"],
    deps = ["//ros2/protobuf:ros2_field_options_proto"],
)
```

Add all four to the `data` list of `proto_to_ros2_msg_tests`.

### 7. `ros2/test/protobuf/tests.py`

Each sized field's generated C++ wraps `n` in a `{ }` block so that multiple
sized fields in the same message do not produce duplicate `const std::uint32_t n`
declarations. The test expectations below encode this contract exactly.

```python
class SizedArrayTests:
    def test_sized_bytes(self, tmp_path):
        fp = _load('sized_bytes_proto', 'sized_bytes.proto')
        out = tmp_path / 'SizedBytes.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/sized_bytes.proto', {})
        assert out.read_text() == """\
# Generated from proto source: ros2/test/protobuf/sized_bytes.proto

uint32 HASH_MAX_SIZE=32
uint8[32] hash
uint32 hash_actual_size
uint8[] raw
"""

    def test_sized_array(self, tmp_path):
        fp = _load('sized_array_proto', 'sized_array.proto')
        msg_type_map = {
            '.ros2.test.protobuf.Point':
                'ros2_test_protobuf_point_proto_ros_msgs/Point',
        }
        out = tmp_path / 'SizedArray.msg'
        proto_to_ros2_msg._convert(
            fp, str(out), f'{_PKG}/sized_array.proto', msg_type_map)
        assert out.read_text() == """\
# Generated from proto source: ros2/test/protobuf/sized_array.proto

uint32 COORDS_MAX_SIZE=3
float32[3] coords
uint32 coords_actual_size
float32[] dynamic
uint32 PTS_MAX_SIZE=4
ros2_test_protobuf_point_proto_ros_msgs/Point[4] pts
uint32 pts_actual_size
"""

    def test_sized_enum(self, tmp_path):
        fp = _load('sized_enum_proto', 'sized_enum.proto')
        out = tmp_path / 'SizedEnum.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/sized_enum.proto', {})
        assert out.read_text() == """\
# Generated from proto source: ros2/test/protobuf/sized_enum.proto

# Kind constants
int32 KIND_NONE=0
int32 KIND_A=1

uint32 KINDS_MAX_SIZE=4
int32[4] kinds
uint32 kinds_actual_size
"""

    def test_array_size_on_singular_scalar_is_error(self, tmp_path):
        fp = _load('sized_bad_proto', 'sized_bad.proto')
        with pytest.raises(SystemExit) as exc:
            proto_to_ros2_msg._convert(
                fp, str(tmp_path / 'SizedBad.msg'),
                f'{_PKG}/sized_bad.proto', {})
        assert 'array_size' in str(exc.value.code)


class SizedArrayConverterTests:
    def _conversions(self, target, proto_file, proto_types_to_ros_pkgs=None):
        fp = _load(target, proto_file)
        message = fp.message_type[0]
        to_ros, from_ros, _ = proto_to_ros2_converter._field_conversions(
            message, f'{_PKG}/{proto_file}', proto_types_to_ros_pkgs or {})
        return to_ros, from_ros

    def test_sized_bytes_converter(self):
        to_ros, from_ros = self._conversions('sized_bytes_proto', 'sized_bytes.proto')
        assert '\n'.join(to_ros) == """\
  {
    const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.hash().size()), 32U);
    ros->hash_actual_size = n;
    std::copy_n(proto.hash().begin(), n, ros->hash.begin());
  }
  ros->raw = std::vector<uint8_t>(proto.raw().begin(), proto.raw().end());"""
        assert '\n'.join(from_ros) == """\
  {
    const std::uint32_t n = std::min(ros.hash_actual_size, 32U);
    proto->set_hash(reinterpret_cast<const char*>(ros.hash.data()), n);
  }
  proto->set_raw(std::string(ros.raw.begin(), ros.raw.end()));"""

    def test_sized_array_converter(self):
        pkg = 'ros2_test_protobuf_point_proto_ros_msgs'
        to_ros, from_ros = self._conversions(
            'sized_array_proto', 'sized_array.proto',
            {'.ros2.test.protobuf.Point': pkg})
        assert '\n'.join(to_ros) == """\
  {
    const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.coords().size()), 3U);
    ros->coords_actual_size = n;
    std::copy_n(proto.coords().begin(), n, ros->coords.begin());
  }
  ros->dynamic.assign(proto.dynamic().begin(), proto.dynamic().end());
  {
    const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.pts().size()), 4U);
    ros->pts_actual_size = n;
    for (std::uint32_t i = 0; i < n; ++i) {
      ros2_test_protobuf_point_proto_ros_msgs::proto_converters::ToRos(proto.pts(i), &ros->pts[i]);
    }
  }"""
        assert '\n'.join(from_ros) == """\
  {
    const std::uint32_t n = std::min(ros.coords_actual_size, 3U);
    proto->mutable_coords()->Assign(ros.coords.begin(), ros.coords.begin() + n);
  }
  proto->mutable_dynamic()->Assign(ros.dynamic.begin(), ros.dynamic.end());
  {
    const std::uint32_t n = std::min(ros.pts_actual_size, 4U);
    proto->clear_pts();
    proto->mutable_pts()->Reserve(n);
    for (std::uint32_t i = 0; i < n; ++i) {
      ros2_test_protobuf_point_proto_ros_msgs::proto_converters::ToProto(ros.pts[i], proto->add_pts());
    }
  }"""

    def test_sized_enum_converter(self):
        to_ros, from_ros = self._conversions('sized_enum_proto', 'sized_enum.proto')
        assert '\n'.join(to_ros) == """\
  {
    const std::uint32_t n = std::min(static_cast<std::uint32_t>(proto.kinds().size()), 4U);
    ros->kinds_actual_size = n;
    for (std::uint32_t i = 0; i < n; ++i) {
      ros->kinds[i] = static_cast<int32_t>(proto.kinds(i));
    }
  }"""
        assert '\n'.join(from_ros) == """\
  {
    const std::uint32_t n = std::min(ros.kinds_actual_size, 4U);
    proto->clear_kinds();
    proto->mutable_kinds()->Reserve(n);
    for (std::uint32_t i = 0; i < n; ++i) {
      proto->add_kinds(static_cast<ros2::test::protobuf::SizedEnum::Kind>(ros.kinds[i]));
    }
  }"""
```

### 8. `ros2/protobuf/README.md`

Add a **Sized arrays** section documenting:
- The `(ros2.msg.array_size)` option, import path, and field number.
- The sized-container model (capacity vs. actual count).
- The emitted triple (`MAX_SIZE` constant, `T[N]` field, `_actual_size` field).
- What `proto_to_ros2_converter.py` does with `_actual_size`.

---

---

## Verification

```
bazel test //ros2/test/protobuf:proto_to_ros2_msg_tests --test_output=streamed
```

All tests in `SizedArrayTests` and `SizedArrayConverterTests` must pass; all
pre-existing tests must continue to pass.
