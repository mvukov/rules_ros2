# Plan: Upgrade rules_ros2 from ROS 2 Jazzy to ROS 2 Lyrical Luth

## Context

rules_ros2 is pinned to `release-jazzy-20250820`. Lyrical Luth (May 2026) is the new LTS, supported until May 2031. The latest release is **`release-lyrical-20260807`**. The goal is to make every Bazel target in `//...` and in `examples/` build and pass tests against Lyrical on our Ubuntu 24.04 environment, with gcc and clang.

### Platform and toolchain facts

These come from the Lyrical supported-platforms page (`ros2_documentation/source/Releases/lyrical/supported-platforms.rst`) and REP 2000.

- **Ubuntu 24.04 Noble is Tier 3 for Lyrical** (EOL 2029-06-01), so building from source is required. That is what Bazel does anyway. Tier 1 is Ubuntu 26.04.
- **Minimum language versions:** C++20, C17, Python 3.12–3.14. The listed Noble versions are Python 3.12.3, EmPY 3.3.4 and NumPy 1.26.4.
- **Compilers:** the docs give no compiler minimum beyond C++20. The devcontainer/CI image (`mvukov/bazel-builder`) has gcc 14.2 and clang 20.1, and both support C++20 fully. No toolchain change is needed.
- **Python:** keep `_PYTHON_VERSION = "3.12"` in [MODULE.bazel](MODULE.bazel). It is the Lyrical minimum and the Noble system version. Keep `empy==3.3.*` and `numpy~=1.26`, which match the Noble column.
- **Middleware:** Cyclone DDS moves from **0.10.5 to 11.0.1**. iceoryx is now pinned to commit `35311bfe…`.

### Preliminary analysis (repos diff Jazzy 20250820 → Lyrical 20260807)

- All 41 mapped repos get bumped. The big jumps are rclcpp 28→32, rclpy 7→10, rcl 9→10, rosidl 4→5, rmw_dds_common 3→6, rmw_cyclonedds 2→4, message_filters 4→7, urdfdom 4→6, urdfdom_headers 1→3 and kdl_parser 2→3.
- **`osrf_pycommon` is no longer in ros2.repos.** `launch` still needs it, so move it out of [ros2_repo_mappings.yaml](repositories/ros2_repo_mappings.yaml) and into a manual `http_archive` (2.1.7) in [repositories.bzl](repositories/repositories.bzl). Otherwise the resolver emits an entry with no url/sha.
- **`urdf` (2.13.2) is now in ros2.repos.** Move it from the manual pin in repositories.bzl into the mappings yaml so the resolver manages it, and keep its patch.
- **`rosidl_rust` (0.4.12) is now in ros2.repos too, but it stays a manual pin in repositories.bzl for now.** Add a note there (`# NOTE: rosidl_rust is in Lyrical ros2.repos (0.4.12); kept manual so all Rust deps are updated together.`). All Rust pieces (ros2_rust, rosidl_runtime_rs, rosidl_rust) get updated together, in sequence, in L7.
- **iceoryx mapping:** drop the hard-coded `strip_prefix: iceoryx-2.0.6`. The resolver will derive `iceoryx-<hash>`.
- **Packages added inside repos:**
  - `rosidl`: `rosidl_buffer`, `rosidl_buffer_backend`, `rosidl_buffer_backend_registry`, `rosidl_buffer_py`.
  - `rmw`: `rmw_security_common`, now needed by `rmw_dds_common` and `rmw_cyclonedds_cpp`.
  - `rcl_logging`: `rcl_logging_implementation`, which `rcl` now depends on.
  - `image_common`: `image_transport_py`.
  - `pluginlib`: `ros2plugin`.
  - `rosbag2`: lz4/zstd cmake modules (not needed).
- **Packages removed:** `actionlib_msgs` from common_interfaces, `libcurl_vendor`, and the rosbag2 vendor packages (unused; we use BCR). `kdl_parser` is now a single package at the repo root, so the paths in [kdl_parser.BUILD.bazel](repositories/kdl_parser.BUILD.bazel) need fixing. `urdfdom_headers` is now at the repo root.
- **API change that affects generated code:** in C++, `uint8[]` message fields are now `rosidl::Buffer<uint8_t>` instead of `std::vector<uint8_t>`. `rosidl_runtime_cpp` depends on `rosidl_buffer`. This hits the generator rules in [ros2/interfaces.bzl](ros2/interfaces.bzl), and also code in examples, foxglove_bridge, rosbag2 and image_transport.
- **Logging backend:** Lyrical's `rcl` links `rcl_logging_implementation`, which dlopens a backend chosen by `RCL_LOGGING_IMPLEMENTATION`. Recommendation: **keep our build-time select** ([rcl.BUILD.bazel:33-37](repositories/rcl.BUILD.bazel#L33-L37)). Expose the selected backend in place of `rcl_logging_implementation`, which has the same `rcl_logging_interface` symbols, so nothing is dlopened. If a shim is needed, follow the pattern of [rmw_implementation_library_path.patch](repositories/patches/rmw_implementation_library_path.patch).
- **Third-party pins outside ros2.repos:** each needs a Lyrical-compatible version.
  - foxglove_bridge 0.7.9 → 0.8.x
  - ros2_rust 0.6.0 → 0.8.0
  - rosidl_runtime_rs 0.5.0 → 0.7.0
  - rcl_logging_syslog: use its `lyrical` branch
  - diagnostics: lyrical-compatible tag
  - xacro, gps_umd and mcap: verify they still build

## Step-by-step plan

### Step 0: Branch and baseline

0. Write this plan to `lyrical_plan.md` in the repo root. Create `lyrical_progress.md` in the repo root as well, with a checklist of steps and layers/packages, the status of each, its commit hash, and notes on blockers and patch rebases. Update it after every package and every commit. **Both files stay uncommitted**: never `git add` them; stage files by explicit path rather than with `git add -A` / `git commit -a`.
1. Check the uncommitted changes in `.bazelrc` and `MODULE.bazel`: commit them, stash them, or keep them. Then `git checkout -b feature/lyrical`.
2. Record a Jazzy baseline: run `bazel test //...` (gcc and `--config=clang`) and save the list of passing targets. Expect flakes from `mcap_tests` and `foxglove_bridge`.
3. **Package-deps analysis on Jazzy (baseline graph):**
   - List the external repos and edges per target: `bazel cquery 'deps(//...)' --output=graph`, or for each repo, `bazel cquery 'kind("cc_library|py_library", @ros2_<repo>//...)'`.
   - Build a repo-level DAG by collapsing labels to `@repo`, then topologically sort it. Save it in the scratchpad (for example `deps_graph.dot` and `build_order.txt`).
   - Cross-check it against the Lyrical `package.xml` deps (fetch them with `gh api` per repo) to add the new edges (rosidl_buffer, rmw_security_common, rcl_logging_implementation, and others).

### Step 1: Global config (commit: "Switch to C++20 / C17 for Lyrical")

- [.bazelrc](.bazelrc) and [examples/.bazelrc](examples/.bazelrc): change `--cxxopt=-std=c++17` to `--cxxopt=-std=c++20` and update the comment ("ROS 2 Lyrical needs min C++20").
- [ros2/cc_opts.bzl](ros2/cc_opts.bzl): change `C_COPTS = ["-std=c11"]` to `["-std=c17"]`. Also check `_c_copts` in [ros2/interfaces.bzl](ros2/interfaces.bzl).
- Rebuild Jazzy `//...` with C++20 to separate C++20-only breakage from Lyrical breakage. This is optional but cheap.

### Step 2: Resolve Lyrical sources (commit: "Bump ROS 2 repos to release-lyrical-20260807")

1. [repositories/repositories.bzl](repositories/repositories.bzl): point the `ros2` http_archive at `release-lyrical-20260807`, with its sha256 (`curl -L … | sha256sum`) and `strip_prefix = "ros2-release-lyrical-20260807"`.
2. Edit [ros2_repo_mappings.yaml](repositories/ros2_repo_mappings.yaml):
   - Remove `osrf_pycommon` and add it as a manual archive in repositories.bzl.
   - Drop the iceoryx `strip_prefix`.
   - Add `urdf`, keeping its patch, and remove its manual pin. Do not touch `rosidl_rust` here; it only gets the NOTE comment.
   - Keep the keys sorted, because `//repositories/private:tests` checks this.
3. Run `bazel run //repositories/private:resolver`. This regenerates [ros2_repositories_impl.bzl](repositories/ros2_repositories_impl.bzl) using [resolver.py](repositories/private/resolver.py), which needs no changes: it handles GitHub hash versions via `is_git_hash`.
4. Update the `use_repo` list in [MODULE.bazel](MODULE.bazel) for any repos that were added or removed.
5. Check every patch in [repositories/patches/](repositories/patches/) with `patch --dry-run` against the new sources. List the ones that no longer apply and fix them in the step for that package.
6. Commit. The tree will not build yet; the commit message should say so.

### Step 3: Bottom-up, one package or layer at a time

For each node in `build_order.txt`:

1. Diff the upstream CMakeLists.txt and package.xml from Jazzy to Lyrical.
2. Update `repositories/<pkg>.BUILD.bazel`: new or removed sources, headers, include paths, generated headers, deps and defines.
3. Rebase or rewrite its patches.
4. Run `bazel build @ros2_<pkg>//...`, then the `bazel test` targets that cover it, with gcc and `--config=clang`.
5. **Commit for each package or small layer**, for example "lyrical: update rcutils BUILD".

The expected order is below. The cquery output from Step 0 is authoritative.

| Layer      | Packages                                                                                                                                                                                          | Notable work                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| L0         | rcutils, ros2_tracing (tracetools), ament_index, console_bridge, urdfdom_headers, iceoryx, cyclonedds                                                                                             | rcutils: new base64/strnlen sources. **cyclonedds 11.0.1:** new CMake config / generated headers (`dds/features.h`, `ddsrt` config) and an iceoryx-at-hash API check.                                                                                                                                                                                                                                                                                                                          |
| L1         | rcpputils, rosidl (runtime_c/cpp, **rosidl_buffer**, buffer_backend, generators, parser, cli), rosidl_dynamic_typesupport, rosidl_typesupport                                                     | rcpputils: thread_name sources. rosidl 5.x: **update generator templates/args in [ros2/interfaces.bzl](ros2/interfaces.bzl)** (Buffer types, type description). Re-check the 4 rosidl patches.                                                                                                                                                                                                                                                                                                 |
| L2         | rmw (+ **rmw_security_common**), rcl_interfaces, unique_identifier_msgs, common_interfaces (drop actionlib_msgs), rosidl_python, rosidl_runtime_py, rpyutils                                      | Interface generation end-to-end: C, C++, Python, introspection.                                                                                                                                                                                                                                                                                                                                                                                                                                |
| L3         | rmw_dds_common, rmw_cyclonedds, rmw_implementation                                                                                                                                                | Rebase rmw_implementation_library_path.patch.                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| L4         | rcl_logging, rcl_logging_syslog (lyrical branch), rcl, libstatistics_collector                                                                                                                    | Decide how to handle rcl*logging_implementation (see Context). Keep the `//ros2:rcl_logging*\*` selects working.                                                                                                                                                                                                                                                                                                                                                                               |
| L5         | rclcpp, class_loader, pluginlib, rclpy, osrf_pycommon, launch, launch_ros                                                                                                                         | rclcpp: regenerate logging/interface headers with [generate*rclcpp*\*.py](repositories/) and rebase the signal-handler patch. rclpy 10: pybind11 sources, new modules. Rebase the launch patch.                                                                                                                                                                                                                                                                                                |
| L6         | message_filters, geometry2, image_common, keyboard_handler, ros2cli, rosbag2, urdfdom, urdf, kdl_parser, orocos_kdl, robot_state_publisher, resource_retriever, diagnostics, xacro, gps_umd, mcap | rosbag2 0.33: rebase its 2 patches. Fix kdl_parser root-layout paths. orocos_kdl: pick a version now that orocos_kdl_vendor is gone.                                                                                                                                                                                                                                                                                                                                                           |
| L-FOXGLOVE | foxglove_bridge, in its own layer and its own commit (larger changes expected)                                                                                                                    | Bump 0.7.9 → 0.8.x, revisit websocketpp `-std=c++17`, ROS_DISTRO string.                                                                                                                                                                                                                                                                                                                                                                                                                       |
| L7         | the Rust stack in sequence: rosidl_runtime_rs → rosidl_rust → ros2_rust                                                                                                                           | Bump the pins, with one commit per Rust repo. Revisit later whether rosidl_rust should move to ros2.repos management. Change the `ROS_DISTRO` "jazzy" → "lyrical" strings in [ros2_rust.BUILD.bazel:68](repositories/ros2_rust.BUILD.bazel#L68), [ros2/rust_interfaces.bzl:162](ros2/rust_interfaces.bzl#L162), [third_party/foxglove_bridge/BUILD.bazel:44](third_party/foxglove_bridge/BUILD.bazel#L44) and [examples/foxglove_bridge/launch.py:29](examples/foxglove_bridge/launch.py#L29). |

### Step 4: rules_ros2 rules and tests (commits per area)

- Run `bazel test //...` in the root workspace and fix [ros2/](ros2/) rules such as `ros2_cpp_binary`, `ros2_test`, launch and bag rules, and `third_party/`.
- Fix test code for C++20 and Lyrical API changes, including `rosidl::Buffer` for `uint8[]` fields and deprecated rclcpp APIs. Keep gtest suite names in `Test<What>` form.
- Run `bazel test //repositories/private:tests`.

### Step 5: Examples workspace (commit)

Run `cd examples && bazel test //...` with gcc and clang. Fix chatter, lifecycle, actions, zero_copy (iceoryx) and foxglove_bridge.

### Step 6: Docs and housekeeping (commit)

- [README.md](README.md): update the default lock to `release-lyrical-20260807`, note C++20, and point to a `feature/jazzy` branch for legacy users if wanted.
- Update [docs/ros2_repos.md](docs/ros2_repos.md) if it names versions.
- Update `MODULE.bazel.lock` files and the renovate config if it references jazzy.
- Run `pre-commit run --all-files`.

## Verification

- `bazel test //...` passes in `.` and `examples/` with gcc and `--config=clang`. This is the CI matrix in [.github/workflows/main.yml](.github/workflows/main.yml), reproduced locally with `--bazelrc=.github/workflows/ci.bazelrc`. Compare against the Step 0 baseline; known flakes are mcap_tests and foxglove_bridge.
- Smoke runs: `bazel run //examples/chatter:...` talker/listener, a launch-based test, a `ros2 bag` record/play test, and the zero_copy example (Cyclone 11 + iceoryx).
- Check that the logging select still works: build with `--@com_github_mvukov_rules_ros2//ros2:rcl_logging_impl=…` for noop, spdlog and syslog.
- `bazel run //repositories/private:resolver` is idempotent, so re-running it gives no diff.
