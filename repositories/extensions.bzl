load("//repositories:repositories.bzl", "ros2_repositories")
load("//repositories:third_party_repositories.bzl", "third_party_repositories")
load("//repositories:bindgen_crates.bzl", "crate_repositories")
load("//repositories:clang_configure.bzl", "clang_configure")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _vendored_repository_impl(ctx):
    repo_name = ctx.attr.repo_name
    
    # Attempt to find the real path of the rules_ros2 module root
    rules_ros2_root = ctx.path(Label("@com_github_mvukov_rules_ros2//:MODULE.bazel")).dirname
    
    # Resolve the physical path using 'pwd -P'
    # We execute this inside the rules_ros2 directory
    res = ctx.execute(["/bin/sh", "-c", "cd '{}' && pwd -P".format(str(rules_ros2_root))])
    
    if res.return_code != 0:
        fail("Failed to resolve rules_ros2 root: {}".format(res.stderr))
        
    real_rules_ros2_root = res.stdout.strip()
    
    # Assume vendor is at <rules_ros2_root>/../vendor
    # We construct the path string manually because ctx.path() objects are restrictive
    # We use basic string manipulation to go up one level
    # real_rules_ros2_root is like /path/to/Perimeta_v2/rules_ros2
    # We want /path/to/Perimeta_v2/vendor
    
    # Find the last slash
    last_slash = real_rules_ros2_root.rfind("/")
    if last_slash == -1:
        fail("Could not parse rules_ros2 root path: {}".format(real_rules_ros2_root))
        
    workspace_root = real_rules_ros2_root[:last_slash]
    vendor_path = "{}/vendor/{}".format(workspace_root, repo_name)
    
    # List files in the vendor directory
    res = ctx.execute(["ls", "-A", vendor_path])
    if res.return_code != 0:
        fail("Failed to list vendor directory {}: {}".format(vendor_path, res.stderr))
    
    files = res.stdout.splitlines()
    
    for f in files:
        f = f.strip()
        if not f: continue
        ctx.symlink("{}/{}".format(vendor_path, f), f)

vendored_repository = repository_rule(
    implementation = _vendored_repository_impl,
    local = True,
    attrs = {
        "repo_name": attr.string(mandatory = True),
    },
)

def _ros2_deps_impl(ctx):
    clang_configure(name = "rules_ros2_config_clang")
    
    crate_repositories()

    http_archive(
        name = "rules_rust_bindgen__bindgen-cli-0.71.1",
        integrity = "sha256-/e0QyglWr9DL5c+JzHGuGmeeZbghbGUfyhe6feisVNw=",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/bindgen-cli/bindgen-cli-0.71.1.crate"],
        strip_prefix = "bindgen-cli-0.71.1",
        build_file = "@rules_rust_bindgen//3rdparty:BUILD.bindgen-cli.bazel",
    )

    repos = [
        "asio",
        "boringssl",
        "console_bridge",
        "curl",
        "cyclonedds",
        "eigen",
        "fmt",
        "foxglove_bridge",
        "iceoryx",
        "libyaml",
        "lz4",
        "mcap",
        "nlohmann_json",
        "orocos_kdl",
        "osrf_pycommon",
        "pybind11",
        "readerwriterqueue",
        "ros2",
        "ros2_ament_index",
        "ros2_class_loader",
        "ros2_common_interfaces",
        "ros2_diagnostics",
        "ros2_geometry2",
        "ros2_gps_umd",
        "ros2_image_common",
        "ros2_kdl_parser",
        "ros2_keyboard_handler",
        "ros2_launch",
        "ros2_launch_ros",
        "ros2_libstatistics_collector",
        "ros2_message_filters",
        "ros2_pluginlib",
        "ros2_rcl",
        "ros2_rcl_interfaces",
        "ros2_rcl_logging",
        "ros2_rcl_logging_syslog",
        "ros2_rclcpp",
        "ros2_rclpy",
        "ros2_rcpputils",
        "ros2_rcutils",
        "ros2_resource_retriever",
        "ros2_rmw",
        "ros2_rmw_cyclonedds",
        "ros2_rmw_dds_common",
        "ros2_rmw_implementation",
        "ros2_robot_state_publisher",
        "ros2_rosbag2",
        "ros2_rosidl",
        "ros2_rosidl_python",
        "ros2_rosidl_runtime_py",
        "ros2_rosidl_runtime_rs",
        "ros2_rosidl_rust",
        "ros2_rosidl_typesupport",
        "ros2_rpyutils",
        "ros2_rust",
        "ros2_tracing",
        "ros2_unique_identifier_msgs",
        "ros2_urdf",
        "ros2_urdfdom",
        "ros2_urdfdom_headers",
        "ros2_xacro",
        "ros2cli",
        "rules_ros2_config_clang",
        "rules_rust_bindgen",
        "rules_rust_bindgen__bindgen-cli-0.71.1",
        "spdlog",
        "sqlite3",
        "tinyxml",
        "tinyxml2",
        "websocketpp",
        "yaml-cpp",
        "zlib",
        "zstd",
    ]

    for repo in repos:
        if repo == "rules_ros2_config_clang":
            continue
        if repo == "rules_rust_bindgen__bindgen-cli-0.71.1":
            continue
        vendored_repository(name = repo, repo_name = repo)

ros2_deps = module_extension(
    implementation = _ros2_deps_impl,
)
