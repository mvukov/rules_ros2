[![CI](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml)

# Bazel Rules for ROS 2

This repository provides Bazel rules and macros to build, run, and test ROS 2 applications using C++, Python, and Rust. It eliminates the need to install ROS 2 system packages (e.g., via `apt`), offering a hermetic and reproducible build environment.

## Features

- **Multi-Language Support**: Build nodes in C++, Python, and Rust.
- **Interface Generation**: Automatic code generation for Messages (`.msg`), Services (`.srv`), and Actions (`.action`) for all supported languages.
- **Launch System**: Define deployments using `ros2_launch` (similar to `ros2 launch`).
- **Testing**: Define integrated ROS 2 tests with `ros2_test`.
- **Middleware**: Uses [Eclipse CycloneDDS](https://cyclonedds.io/) with optional shared memory support via [Eclipse Iceoryx](https://iceoryx.io/) ("zero-copy").
- **Tools & Utilities**:
  - `foxglove_bridge` for visualization.
  - CLI tools:
    - `ros2_bag`: for handling rosbags.
    - `ros2_lifecycle`: for handling node lifecycle.
    - `ros2_node`: for handling nodes.
    - `ros2_param`: for handling parameters.
    - `ros2_service`: for handling services.
    - `ros2_topic`: for handling topics.
  - `xacro` conversion support.

## Prerequisites

- **Bazel**: [Install Bazel](https://bazel.build/install) (Version 7.0+ recommended).
- **Python**: Python 3.10+ installed on your system.
- **C++ Compiler**: Clang (recommended) or GCC.
- **OS**:
  - Linux (Ubuntu 22.04 tested)
  - macOS (Tested on Apple Silicon)

**Note:** You do *not* need to install ROS 2 via your system package manager.

## Installation

Add the following to your `MODULE.bazel` to depend on `rules_ros2`:

```bazel
bazel_dep(name = "rules_ros2", version = "0.0.0")
# If using a specific commit/version, use git_override or local_path_override as needed.
```

## Quick Start

Check out the [examples/](examples) directory for complete, runnable examples.

### Running Examples

You can run examples in two ways:

1.  **From your project root** (if `rules_ros2` is an external dependency):
    ```bash
    bazel run @rules_ros2//examples/chatter:chatter
    ```

2.  **As a standalone workspace** (if cloning this repo directly):
    ```bash
    cd examples
    bazel run //chatter --experimental_isolated_extension_usages
    ```

### Common Commands

- **Run a launch file:**
  ```bash
  bazel run //path/to:launch_target
  ```

- **Run a single node:**
  ```bash
  bazel run //path/to:node_target
  ```

- **Run tests:**
  ```bash
  bazel test //path/to:test_target
  ```

- **List active topics:**
  ```bash
  bazel run @rules_ros2//:ros2_topic -- list
  ```

## Examples Overview

| Example | Description | Language |
| :--- | :--- | :--- |
| [Chatter](examples/chatter) | Basic pub/sub communication. | C++, Python, Rust |
| [Lifecycle](examples/lifecycle) | Managed lifecycle nodes. | C++ |
| [Actions](examples/actions) | Action server and client (Fibonacci). | C++, Python |
| [Foxglove Bridge](examples/foxglove_bridge) | Visualization with Foxglove Studio. | C++ |
| [Zero Copy](examples/zero_copy) | Shared memory transport with Iceoryx. | C++, Rust |

## Configuration

### Logging Backend
Switch logging backends using build flags:
- **spdlog (default):** `--@com_github_mvukov_rules_ros2//ros2:rcl_logging_impl=spdlog`
- **syslog:** `--@com_github_mvukov_rules_ros2//ros2:rcl_logging_impl=syslog`

### Shared Memory (Zero Copy)
Enable shared memory for CycloneDDS:
```bash
bazel run //target -- @cyclonedds//:enable_shm=True
```

## Troubleshooting

- **Relative Path Issues:** Ensure your launch files resolve paths relative to `__file__` or use `runfiles` library to locate artifacts, especially when running from different workspace contexts.
- **macOS Builds:** Ensure you have Xcode Command Line Tools installed. Some Linux-specific paths might need adjustment.

## Adaptations & Fork Details

This repository is based on [mvukov/rules_ros2](https://github.com/mvukov/rules_ros2) but has been significantly adapted for:

1.  **Bazel 8+ & Bzlmod**:
    - Migrated from the deprecated `WORKSPACE` system to `MODULE.bazel` (Bzlmod).
    - Dependencies are managed via Bazel modules and extensions, ensuring forward compatibility.

2.  **Submodule Integration**:
    - Adapted to work as a subdirectory/module within a larger root workspace (`Perimeta_v2`).
    - Supports dual-mode execution: can be run as a standalone workspace (via `examples/`) or as an external dependency from the project root.

3.  **Cross-Platform Fixes**:
    - Enhanced macOS support, including fixes for relative paths in launch files and build configurations.
    - Updated `launch.py` scripts to robustly resolve executable paths relative to `__file__`, solving issues when invoked from different workspace roots.

4.  **Vendored Dependencies**:
    - Dependencies like `curl` have been patched to work correctly in this integrated environment.

## Alternatives

- [`ApexAI/rules_ros`](https://github.com/ApexAI/rules_ros/)
- [`RobotLocomotion/drake-ros`](https://github.com/RobotLocomotion/drake-ros)