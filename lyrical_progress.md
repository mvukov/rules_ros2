# ROS 2 Lyrical upgrade — progress

Plan: [lyrical_plan.md](lyrical_plan.md). Branch: `feature/lyrical` (from `5d1fc8c`).
Target release: `release-lyrical-20260807`.
NOTE: this file and lyrical_plan.md were kept uncommitted during the upgrade and committed at the end.

Legend: [ ] todo · [~] in progress · [x] done · [!] blocked

## Step 0: Branch and baseline

- [x] Plan/progress files created in the repo root
- [x] Working tree clean (the earlier .bazelrc/MODULE.bazel changes were already committed as 5d1fc8c); branch `feature/lyrical` created
- [x] Jazzy baseline `bazel test //...` (gcc): 38/38 pass (target list in scratchpad `baseline_gcc_targets.txt`)
- [-] Jazzy baseline (clang): skipped at the user's request; the gcc baseline is the reference
- [x] cquery repo-level deps graph + topo order (scratchpad: `deps_graph.dot`, `build_order.txt`, `build_order_final.txt`)
- [x] Cross-check with Lyrical package.xml deps: no ordering changes (see log)

## Step 1: C++20 / C17

- [x] .bazelrc → `-std=c++20` (examples/.bazelrc is a symlink to it: never `sed -i` it, that replaces the link with a file)
- [x] ros2/cc_opts.bzl → `-std=c17` (interfaces.bzl `_c_copts`/`_py_ext_c_copts` use C_COPTS). Dropped local `-std=c++17` in class_loader and `-std=c11` in rmw. Kept `-std=c++17` on foxglove_bridge_base (websocketpp only, no ROS headers): revisit at L11
- [x] Jazzy builds with C++20: 38/38 pass (foxglove_bridge FLAKY, known). **Committed 3de38df**

## Step 2: Resolve Lyrical sources

- [x] ros2 http_archive → release-lyrical-20260807 (sha 6906704f…)
- [x] mappings: removed osrf_pycommon (now a manual pin in repositories.bzl), dropped the iceoryx strip_prefix, added urdf (with its patch); dropped the rclcpp patch
- [x] rosidl_rust: NOTE comment only (kept manual)
- [x] Resolver run: 40 archives. Gotcha: had to hand-delete the stale osrf_pycommon block from the generated impl first (duplicate repo name breaks extension eval)
- [x] MODULE.bazel use_repo: no change needed (same repo names)
- [x] Patch dry-run results (against Lyrical tarballs):
  - OK as-is: rosbag2_relax_plugin_errors, rosbag2_load_plugin_cli_extensions, rosidl_fix_rosidl_typesupport_introspection_generator, rosidl_typesupport_generate_true_c_code, urdf_plugin_loader_fix (vs urdf 2.13.2)
  - REBASE: launch_no_log_dir_latest_symlink (upstream still creates `latest`, now type-annotated), rmw_implementation_library_path (1/2 hunks), rosidl_rm_unnecessary_asserts (3 hunks), rosidl_python_fix_imports (3 files)
  - DROP: rclcpp_do-not-allocate-in-signal-handler (fixed upstream: the log moved to deferred_signal_handler)
  - Rust patches (ros2*rust*\*): checked at L-RUST
- [x] `bazel fetch` of all repos: all OK except the 4 REBASE patches (launch, rmw_implementation, rosidl, rosidl_python). Non-use_repo repos need `--repo=@@+non_module_deps+<name>`
- [x] `//repositories/private:tests` pass. **Committed 9c4ac01**

## Step 3: Bottom-up packages

Order from the cquery analysis (scratchpad `build_order_final.txt`). Packages within a layer are independent of each other.
| Layer | Package | Status | Commit | Notes |
|---|---|---|---|---|
| L0 | rcutils | [x] gcc+clang | 3fa8ed5 | logging*macros.h now checked in upstream: dropped empy generation and generate_rcutils_logging_macros.py. rclcpp still refers to the removed :logging_generator (fix at L10) |
| L0 | ament_index | [x] gcc+clang | - | builds unchanged |
| L0 | console_bridge | [x] gcc+clang | - | builds unchanged |
| L0 | urdfdom_headers | [x] gcc+clang | - | builds unchanged (root layout already matched) |
| L0 | ros2_tracing | [x] gcc+clang | e9d8ac7 | visibility_control.hpp → .h |
| L0 | rpyutils | [x] gcc+clang | - | builds unchanged |
| L0 | keyboard_handler | [x] gcc+clang | - | builds unchanged |
| L0 | osrf_pycommon | [x] gcc+clang | - | builds unchanged (manual pin) |
| L0 | mcap | [x] gcc+clang | - | builds unchanged |
| L0 | orocos_kdl | [x] gcc+clang | - | builds unchanged |
| L0 | iceoryx | [x] gcc+clang | - | builds unchanged at commit 35311bf |
| L0 | cyclonedds | [x] gcc+clang | f21fe76, dcec124 | ENABLE_SHM → ENABLE_ICEORYX (PSMX plugin, statically linked into libddsc.a); pinned ENABLE_ICEORYX2/ENABLE_TCP_TLS OFF. Built with SHM on and off |
| L1 | rcpputils | [x] gcc+clang | - | builds unchanged (src glob covers new thread_name.cpp) |
| L1 | rosidl(+rosidl_buffer) | [x] gcc+clang | e768b54, a60548a | added rosidl_buffer lib; rosidl_pycommon → glob (interface_base_classes.py, used by rclpy); rebased asserts patch. Generators get verified at L2. rosidl_buffer_backend{,\_registry} not built. a60548a: +rosidl_buffer_py (pybind11 ext) |
| L1 | launch | [x] gcc+clang | 38d0d20 | rebased symlink patch; +pyyaml, +typing_extensions (new pip dep, lock: typing-extensions==4.16.0). Import smoke test OK |
| L1 | resource_retriever | [x] gcc+clang | 26bf3e8 | compiled-in retriever plugins (curl, filesystem) |
| L1 | urdfdom | [x] gcc+clang | 1d09b40 | -twist.cpp -urdf_model_state.cpp +urdf_sensor.cpp |
| L1 | xacro | [x] gcc+clang | - | builds unchanged (py; runtime check at Step 4) |
| L2 | class_loader | [x] gcc+clang | - | builds unchanged |
| L2 | rcl_logging | [x] gcc+clang | 6b702cc | noop src moved; rcl_logging_implementation (dlopen shim) NOT built: build-time select kept, backends export the same 4 rcl_logging_external*_ symbols. checked at L8: not called |
| L2 | rcl*logging_syslog | [x] gcc+clang | f5fc75f | pinned lyrical branch de09bdb; latent bug fixed: was ros2_c_library for a .cpp (clang rejects -std=c17 on C++) |
| L2 | rosidl_dynamic_typesupport | [x] gcc+clang | - | builds unchanged |
| L2 | rosidl_typesupport | [x] gcc+clang | - | builds unchanged (patch OK); generators verified with interfaces |
| L2 | rosidl_python | [x] gcc+clang | 101b6b7 | patch rebased via script (dedent skips empy @{ }@ blocks!). Generated py needs rosidl_pycommon + rosidl_buffer_py at runtime: wire into interfaces.bzl |
| L2 | rosidl_runtime_py | [x] gcc+clang | 5b52701 | +rosidl_buffer_py dep (top-level import) |
| L3 | pluginlib | [x] gcc+clang | - | builds unchanged |
| L3 | rmw(+rmw_security_common) | [x] gcc+clang | 6796e0c | added rmw_security_common |
| L3 | unique_identifier_msgs | [x] gcc+clang | - | builds unchanged (C generator OK) |
| L4 | rcl_interfaces | [x] gcc+clang | c39d195 | rosgraph_msgs → +rcl_interfaces dep; py_ros2_interface_library +rosidl_pycommon +rosidl_buffer_py. Smoke-tested py msgs (typesupport import) + C++ msgs (uint8[] → rosidl::Buffer) on gcc+clang. NOTE: C++ generation only runs via downstream cpp_ros2_interface_library, so `@repo//...` builds do not exercise it |
| L4 | urdf | [x] gcc+clang | abb144d | headers .h → .hpp (glob); urdfdom_compatibility.h version 3.0.0 |
| L5 | common_interfaces | [x] gcc+clang | - | builds unchanged (actionlib_msgs was never in BUILD) |
| L5 | kdl_parser | [x] gcc+clang | 68a584e | root layout; orocos_kdl pin comment updated (vendor pkg gone) |
| L5 | rmw_dds_common | [x] gcc+clang | - | builds unchanged; rmw_security_common is only re-exported upstream (added to rmw_cyclonedds directly) |
| L6 | rmw_cyclonedds | [x] gcc+clang | 39f0ebd | cdds_version.hpp (11.0 → new code paths, SHM via PSMX, no iceoryx_binding_c); +dyntype_helper.c; +rmw_security_common. Built with SHM on/off. Upstream RMW_VERSION*_ defines skipped: RMW_VERSION_GTE is unused (Jazzy did not set them either) |
| L7 | rmw_implementation | [x] gcc+clang | 03ee59d | patch rebased (ament_index API). Runtime smoke test: loads rmw_cyclonedds_cpp |
| L8 | rcl | [x] gcc+clang | 9d007f7 | rcl_yaml_param_parser: \_GNU_SOURCE (newlocale/uselocale hidden under strict c17; upstream CMake uses gnu ext). Logging: upstream rcl supports direct backend linking → our select is fine; rcl never calls unload_logging_library (L2 TODO closed) |
| L9 | libstatistics_collector | [x] gcc+clang | - | builds unchanged |
| L9 | rclpy | [x] gcc+clang | 05892ed | +py_lifecycle_msgs +rosidl_pycommon +typing_extensions. E2E smoke: rclpy pub/sub over Cyclone 11 OK (gcc+clang) |
| L10 | rclcpp | [x] gcc+clang | 9e921ae | logging.hpp checked in upstream → dropped generation (+ logging_macros.bzl, generate_rclcpp_logging_macros.py); excluded dead create_generic_service.cpp (not in upstream CMake). Runtime smoke: EventsCBGExecutor pub/sub + srv with uint8[] Buffer response (gcc+clang) |
| L10 | ros2cli | [x] gcc+clang | 85f614e | +rosidl_adapter for ros2interface; import-all-modules smoke OK (ros2pkg create needs ament_copyright: pre-existing) |
| L11 | launch_ros | [x] gcc+clang | - | builds unchanged (py; runtime via Step 4 launch tests) |
| L11 | message_filters | [x] gcc+clang | 2e57299 | .h headers dropped → .hpp glob; +cpp_std_msgs; py package split → glob. Py import smoke OK |
| L11 | rosbag2 | [x] gcc+clang | 630187b | cpp src list updated (+action_utils, +transient_local cache, -typesupport_helpers); transport +rclcpp_action +rcl; py info +action/utils files. Smoke: mcap+sqlite3 write/read, ros2bag imports (gcc+clang). NOTE: smoke tests using plugins need ros2_py_test (sets up ament) |
| L11 | diagnostics | [x] gcc+clang | - | builds unchanged |
| L11 | gps_umd | [x] gcc+clang | - | builds unchanged |
| L12 | geometry2 | [x] gcc+clang | - | builds unchanged; src + header lists verified against upstream CMake |
| L12 | image_common | [x] gcc+clang | b6cd06e | +publisher_plugin.cpp +subscriber_plugin.cpp +node_interfaces.hpp, tinyxml2 impl dep; image_transport_py NOT built |
| L13 | robot_state_publisher | [x] gcc+clang | - | builds unchanged (glob covers sources) |
| L-FOXGLOVE | foxglove_bridge | [x] gcc+clang | 4234405, 03ffbd2, 3829c10 | User chose foxglove-sdk 3.6.3 + prebuilt C SDK 0.30.2 (x86_64/aarch64, upstream sha pins), REMOTE_ACCESS OFF (static libfoxglove.a). New deps: rosx_introspection 2.3.0, rapidjson (BCR). Helper ament_version_header() for ament_index_cpp/rclcpp version.h. Ament setup now also exposes lib<pkg>\_\_rosidl_typesupport_introspection_cpp.so (generic service clients). Upstream smoke tests 26/26 pass. NOTE: params.py still keeps the 'services' capability disabled (old TBD); it now works and could be enabled |
| L-RUST | rosidl_runtime_rs | [x] gcc+clang | 30eb77f | 0.7.0 via #669; Lyrical delta: ros_distro=lyrical + C ABI probe rust_test (18/18, sizes/offsets match Lyrical C structs), quickcheck |
| L-RUST | rosidl_rust | [x] gcc+clang | 5dff7bd | 0.5.0 via #669; Lyrical delta: generator ROS_DISTRO=lyrical. Smoke: generated msgs round-trip through RMW |
| L-RUST | ros2_rust | [x] gcc+clang | e4f4668 | 0.8.0 via #669 (direct IDL crate imports, no ros_env); Lyrical delta: ros_distro=lyrical. //ros2/test/rust 4/4 |

## Step 4: rules_ros2 rules and tests

- [x] `bazel test //...` (gcc): same 38 targets as the Jazzy baseline; 36/38 at first, rosbag mcap/sqlite3 tests failed (rosbag2 0.33 behavior changes), fixed in 451e6cc → 38/38
- [x] `bazel test //...` (clang): 38/38 pass, same target set as baseline
- [x] `//repositories/private:tests` (part of //...)

## Step 5: Examples

- [x] examples `bazel test //...` (gcc): 2/2 (chatter, lifecycle)
- [x] examples `bazel test //...` (clang): 2/2. Also built //actions, //foxglove_bridge, //lifecycle and the non-Rust //chatter targets (gcc+clang); foxglove example run: robot_state_publisher + bridge 3.6.3 OK, clean shutdown

## Step 6: Docs and housekeeping

- [x] README, docs/ros2_repos.md (7defd16); MODULE.bazel.lock files are gitignored; renovate.json5 has no distro refs
- [x] `pre-commit run --all-files` passes (rustfmt needs `repositories/rust/rustfmt/generate.sh` first, as in CI)

## Log / notes

- 2026-11-06: Preliminary analysis done during planning (see the Context section of the plan).
- 2026-11-06: Lyrical package.xml deps collected for 171 packages in the 41 mapped repos. Package-level topo order is in
  `/tmp/claude-1000/-workspaces-rules-ros2/91279d19-2bf5-4990-8ad8-9b25dda5b954/scratchpad/lyrical_pkg_order.txt`
  (raw data: `.../scratchpad/lyrical_pkgs.json`).
  Caveats: the repo-level graph has cycles (rcl → ros2_tracing → rclcpp via tracetools_launch and other test/tooling pkgs), so the sort is per package with test/tooling pkgs excluded.
  Edges into repos outside our mappings (e.g. rosidl_defaults) are missing, so this is a cross-check only; Bazel cquery decides the order.
- 2026-11-06: Jazzy baseline `bazel test //...` (gcc) started in the background; log at
  `/tmp/claude-1000/-workspaces-rules-ros2/91279d19-2bf5-4990-8ad8-9b25dda5b954/scratchpad/baseline_gcc.log`.
- 2026-11-06: Clang baseline stopped (user: the gcc baseline is enough).
- 2026-11-06: cquery analysis. `deps(//...)` alone misses repos: `--noimplicit_deps` drops the generator aspects' private attrs (e.g. rosidl_python), and some repos are reachable only via selects or examples.
  Final graph = `deps(//...)` + `deps(@<each of the 45 ext ROS repos>//...)`, implicit and tool deps included, collapsed to repos. The result is acyclic.
  Added by hand: osrf_pycommon (→ launch, launch_ros), rcl_logging_syslog (via rcl's logging select), gps_umd (→ common_interfaces). The iceoryx → cyclonedds/rmw_cyclonedds edge exists only under `enable_shm_on`.
- 2026-11-06: Lyrical cross-check: all new cross-repo edges point downward, so the order is unchanged. Exception: rosidl_buffer_backend{,\_registry} → rmw/pluginlib (repo-level cycle). Nothing we build uses them (only Fast-DDS/Zenoh), so they are deferred; build only rosidl_buffer in L1.
- 2026-11-06: GOTCHA: Bazel's built-in patcher rejects multi-file patches without `diff --git` headers (CONTENT_DOES_NOT_MATCH_TARGET), even when GNU patch and git apply accept them. Always generate patches with `git diff` in a scratch git repo.
- 2026-11-06: GOTCHA: py_library targets "build" even with missing deps or modules. For Python packages, check imports against BUILD deps and run an import smoke test (temporary //zz_smoke package, never committed).
- 2026-11-06: User: defer foxglove_bridge to its own layer (L-FOXGLOVE, after L13 and before L-RUST) with a separate commit; larger changes are expected.
- 2026-11-06: BUG found at L9 via runtime smoke test: rmw_create_node aborted ("free(): invalid pointer" in the sandbox, bus error outside). ASan (`--copt=-fsanitize=address --linkopt=-fsanitize=address`, ASAN_OPTIONS=detect_odr_violation=0) pointed at an uninitialized sertype_rmw::type_information. Root cause: Cyclone was built with ENABLE_TYPE_DISCOVERY=OFF, so dds_create_topic_descriptor(GLOBAL) fails, and rmw_cyclonedds 4 ignores the error. Fixed in dcec124 (type discovery ON).
- NOTE: ASan also reports a pre-existing ODR violation: librmw_cyclonedds.so statically embeds rcutils (and other libs) next to the shared copies in fastbuild tests. Not new in Lyrical; left as-is.
- LESSON: after each layer with runtime code, run a small runtime smoke test (rmw/rcl/rclpy); compile success is not enough.
- 2026-11-07: Resumed after the user's stop; committed rosbag2.
- 2026-11-07: L-FOXGLOVE investigation. 0.7.9 does not compile against rclcpp 32 (get_typesupport_handle, GenericClient overrides, AsyncParametersClient ctor). The ros-foxglove-bridge repo is now "for ROS 1"; 0.8.5 still uses the removed APIs. Lyrical rosdistro ships foxglove_bridge 3.6.3 from foxglove-sdk (ros/src/foxglove_bridge): it FetchContents a prebuilt (Rust-built) Foxglove C SDK zip per platform and needs rosx_introspection. Waiting on a user decision: A) sdk 3.6.3 + prebuilt SDK, B) sdk 3.6.3 + SDK built from source with rules_rust, C) 0.8.5 + patch.
- Side findings from rosdistro lyrical: orocos_kdl_vendor 0.8.0 exists there (could re-derive the orocos_kdl pin); rosidl_rust is 0.5.0 in rosdistro vs 0.4.12 in ros2.repos (relevant for L-RUST).
- 2026-11-07: L-FOXGLOVE done (user decision: SDK 3.6.3 + prebuilt SDK; foxglove before Rust). Root cause of failing service tests: the ament setup only exposed lib<pkg>\_\_rosidl_typesupport_cpp.so; the bridge also loads ...\_introspection_cpp.so. Fixed in 4234405 (pre-existing limitation, also on Jazzy).
- 2026-11-07: Found that PR #669 (ros2_rust 0.8.0, rosidl_rust 0.5.0, rosidl_runtime_rs 0.7.0 for Jazzy) had landed on main. Per the user: dropped my Rust commits (af896af, 9f1b377) and the uncommitted rclrs WIP (kept on branch backup/lyrical-rust-wip + stash "lyrical rclrs WIP (dropped)"), reset to 3829c10, then merged origin/main (d858ffc, clean; includes #669 and googletest #673). L-RUST is now only the Lyrical delta on top of #669.
- 2026-11-07: L-RUST done on top of #669. futures: lock resolves 0.3.34 (try_recv OK), no spec change needed. No 'jazzy' strings left in the root workspace.
- 2026-11-07: PRE-EXISTING (also on main, not Lyrical-related): `bazel build //...` in examples/ fails for the Rust chatter nodes, //chatter:bag and //zero_copy: rules_ros2's crate.from_cargo has no `lockfile` (crate_universe rejects that in a non-root module), and examples/MODULE.bazel doesn't use_repo ros2_rosbag2/iceoryx/googletest. CI misses it because `test --build_tests_only`. Not fixed; flagged to the user.

## Final notes / open items

Status (2026-11-07): all steps done. Root workspace `bazel test //...` 38/38 (gcc+clang), examples tests 2/2 (gcc+clang), pre-commit clean.

Decisions for the maintainer:

- PRE-EXISTING, also on main: `bazel build //...` in examples/ fails for the Rust chatter nodes, //chatter:bag and //zero_copy. rules_ros2's crate.from_cargo has no `lockfile` (crate_universe rejects that in a non-root module), and examples/MODULE.bazel doesn't use_repo ros2_rosbag2/iceoryx/googletest. CI misses it because of `test --build_tests_only`. Probably a separate PR.
- No `feature/jazzy` branch exists (README points to feature/humble and feature/jazzy_workspace only). Consider creating one from main before merging this, and add a README note.
- Branch history contains two non-Lyrical commits at its base: 5d1fc8c (don't force -fuse-ld=lld; not on main yet) and 56a8feb (googletest bump, same change as #673 on main). Decide whether they ship with this PR.
- foxglove_bridge services now work (see 4234405) but the 'services' capability is still disabled in third_party/foxglove_bridge/params.py (old TBD).
- Cleanup: delete branch backup/lyrical-rust-wip and the stash "lyrical rclrs WIP (dropped)" when no longer needed.

Known limitations / follow-ups:

- Not built: image_transport_py, rosidl_buffer_backend{,\_registry} (only used by Fast-DDS/Zenoh buffer backends). rcl_logging_implementation (runtime-selected logging) is intentionally not built; the build-time logging select is kept.
- foxglove_bridge is built with REMOTE_ACCESS OFF (static prebuilt Foxglove C SDK 0.30.2); remote access could become a build flag later.
- orocos_kdl is now pinned manually (orocos_kdl_vendor is gone in Lyrical); rosdistro lyrical has orocos_kdl_vendor 0.8.0 if the pin should be re-derived.
- Version-dependent values to keep in sync on future bumps: cdds*version.hpp (rmw_cyclonedds.BUILD.bazel), urdfdom_compatibility.h (urdf.BUILD.bazel), RESOURCE_RETRIEVER_VERSION*\* (foxglove_bridge.BUILD.bazel), ament_version_header() versions (ament_index, rclcpp), ros_distro cfgs (rosidl_runtime_rs, ros2_rust, rust_interfaces.bzl).
- ASan reports a pre-existing ODR violation: librmw_cyclonedds.so statically embeds rcutils (and others) next to the shared copies in fastbuild tests.
- Scratchpad paths mentioned above (/tmp/claude-1000/...) are session-local and may no longer exist.
