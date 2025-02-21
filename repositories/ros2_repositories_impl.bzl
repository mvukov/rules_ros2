# This file is automatically generated.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros2_repositories_impl():
    maybe(
        http_archive,
        name = "ros2_ament_index",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ament_index.BUILD.bazel",
        sha256 = "e66896e255653508cb2bdecd7789f8dd5a03d7d2b4a1dd37445821a5679c447c",
        strip_prefix = "ament_index-1.4.0",
        url = "https://github.com/ament/ament_index/archive/refs/tags/1.4.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_class_loader",
        build_file = "@com_github_mvukov_rules_ros2//repositories:class_loader.BUILD.bazel",
        sha256 = "a85a99b93fcad7c8d9686672b8e3793200b1da9d8feab7ab3a9962e34ab1f675",
        strip_prefix = "class_loader-2.2.0",
        url = "https://github.com/ros/class_loader/archive/refs/tags/2.2.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_common_interfaces",
        build_file = "@com_github_mvukov_rules_ros2//repositories:common_interfaces.BUILD.bazel",
        sha256 = "d4aeb9f5aa2d5af9938ac4e32c6b7878586096951036c08f1e46fcacdc577c97",
        strip_prefix = "common_interfaces-4.2.4",
        url = "https://github.com/ros2/common_interfaces/archive/refs/tags/4.2.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "cyclonedds",
        build_file = "@com_github_mvukov_rules_ros2//repositories:cyclonedds.BUILD.bazel",
        sha256 = "ec3ec898c52b02f939a969cd1a276e219420e5e8419b21cea276db35b4821848",
        strip_prefix = "cyclonedds-0.10.5",
        url = "https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/0.10.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_geometry2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:geometry2.BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:geometry2_fix-use-after-free-bug.patch"],
        sha256 = "5c273ff836ab9268c01ad240e0d31aca6765b44c3759fce0e87b700381feddfd",
        strip_prefix = "geometry2-0.25.9",
        url = "https://github.com/ros2/geometry2/archive/refs/tags/0.25.9.tar.gz",
    )

    maybe(
        http_archive,
        name = "iceoryx",
        strip_prefix = "iceoryx-2.0.5",
        build_file = "@com_github_mvukov_rules_ros2//repositories:iceoryx.BUILD.bazel",
        sha256 = "bf6de70e3edee71223f993a29bff5e61af95ce4871104929d8bd1729f544bafb",
        url = "https://github.com/eclipse-iceoryx/iceoryx/archive/refs/tags/v2.0.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_image_common",
        build_file = "@com_github_mvukov_rules_ros2//repositories:image_common.BUILD.bazel",
        sha256 = "0433ed59cb813f14072c83511889d6950af0c223e346cd7ff95916274a3135cd",
        strip_prefix = "image_common-3.1.9",
        url = "https://github.com/ros-perception/image_common/archive/refs/tags/3.1.9.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_kdl_parser",
        build_file = "@com_github_mvukov_rules_ros2//repositories:kdl_parser.BUILD.bazel",
        sha256 = "f28da9bd7eaa8995f4b67bc9c8321d7467043aa43e01b918aa239b8b9971bf56",
        strip_prefix = "kdl_parser-2.6.4",
        url = "https://github.com/ros/kdl_parser/archive/refs/tags/2.6.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_keyboard_handler",
        build_file = "@com_github_mvukov_rules_ros2//repositories:keyboard_handler.BUILD.bazel",
        sha256 = "36e64e9e1927a6026e1b45cafc4c8efd32db274bfab5da0edd273a583f3c73f4",
        strip_prefix = "keyboard_handler-0.0.5",
        url = "https://github.com/ros-tooling/keyboard_handler/archive/refs/tags/0.0.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_launch",
        build_file = "@com_github_mvukov_rules_ros2//repositories:launch.BUILD.bazel",
        sha256 = "16c29a3774ed13e09195c9f3d58f4199fa0913a324b8e67f3de2a2da676ce4c7",
        strip_prefix = "launch-1.0.7",
        url = "https://github.com/ros2/launch/archive/refs/tags/1.0.7.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_launch_ros",
        build_file = "@com_github_mvukov_rules_ros2//repositories:launch_ros.BUILD.bazel",
        sha256 = "fa7f6b4e32260629ea8752a50f3d97650fe79b589255bc6cd20b0f08d0cfc3f1",
        strip_prefix = "launch_ros-0.19.8",
        url = "https://github.com/ros2/launch_ros/archive/refs/tags/0.19.8.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_libstatistics_collector",
        build_file = "@com_github_mvukov_rules_ros2//repositories:libstatistics_collector.BUILD.bazel",
        sha256 = "f16eb49c77a37db2b5344a6100d9697b19a55692e36118fb28817089a8d34351",
        strip_prefix = "libstatistics_collector-1.3.4",
        url = "https://github.com/ros-tooling/libstatistics_collector/archive/refs/tags/1.3.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_message_filters",
        build_file = "@com_github_mvukov_rules_ros2//repositories:message_filters.BUILD.bazel",
        sha256 = "fd64677763d15583a8f5efcdf45dd43548afb3f4e4bf1fb79eed55351d6b983b",
        strip_prefix = "message_filters-4.3.5",
        url = "https://github.com/ros2/message_filters/archive/refs/tags/4.3.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "osrf_pycommon",
        build_file = "@com_github_mvukov_rules_ros2//repositories:osrf_pycommon.BUILD.bazel",
        sha256 = "a5c57a1021d1620cfe4620c4f1611e040de86e7afcce53509e968a4098ce1fa2",
        strip_prefix = "osrf_pycommon-2.1.4",
        url = "https://github.com/osrf/osrf_pycommon/archive/refs/tags/2.1.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_pluginlib",
        build_file = "@com_github_mvukov_rules_ros2//repositories:pluginlib.BUILD.bazel",
        sha256 = "74188b886f9bbe7e857d21f3bb50b480e7c0e5adab97c10563dc17013600198b",
        strip_prefix = "pluginlib-5.1.0",
        url = "https://github.com/ros/pluginlib/archive/refs/tags/5.1.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rcl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl.BUILD.bazel",
        sha256 = "81519ac2fff7cd811604514e64f97c85933b7729e090eb60a6278355ed30f13f",
        strip_prefix = "rcl-5.3.9",
        url = "https://github.com/ros2/rcl/archive/refs/tags/5.3.9.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rcl_interfaces",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl_interfaces.BUILD.bazel",
        sha256 = "e267048c9f78aabed4b4be11bb028c8488127587e5065c3b3daff3550df25875",
        strip_prefix = "rcl_interfaces-1.2.1",
        url = "https://github.com/ros2/rcl_interfaces/archive/refs/tags/1.2.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rcl_logging",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl_logging.BUILD.bazel",
        sha256 = "f711a7677cb68c927650e5e9f6bbb5d013dd9ae30736209f9b70f9c6485170f6",
        strip_prefix = "rcl_logging-2.3.1",
        url = "https://github.com/ros2/rcl_logging/archive/refs/tags/2.3.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rclcpp",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rclcpp.BUILD.bazel",
        patch_cmds = ["patch"],
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rclcpp_fix-maybe-uninitialized-warning.patch", "@com_github_mvukov_rules_ros2//repositories/patches:rclcpp_ts_libs_ownership.patch"],
        sha256 = "f2102798b3fd7c11eba2728b35f5aca34add9acc7beb42d0a7e9cfcda12eea3d",
        strip_prefix = "rclcpp-16.0.11",
        url = "https://github.com/ros2/rclcpp/archive/refs/tags/16.0.11.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rclpy",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rclpy.BUILD.bazel",
        sha256 = "2dadc5b7f05d3993c487a8e721e612d62e82b96fa7d243ccd84f048b1a123a41",
        strip_prefix = "rclpy-3.3.15",
        url = "https://github.com/ros2/rclpy/archive/refs/tags/3.3.15.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rcpputils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcpputils.BUILD.bazel",
        sha256 = "40554ef269f40e242175c3f17ae88e77d2bd1768eb4c5a8d0d01b94f59d28948",
        strip_prefix = "rcpputils-2.4.4",
        url = "https://github.com/ros2/rcpputils/archive/refs/tags/2.4.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rcutils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcutils.BUILD.bazel",
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rcutils_fix-setting-allocator-to-null.-478.patch"],
        sha256 = "b64c3077162bc845a7c410180bc6c78e63e3a7562285b74c0982eee101ea0f28",
        strip_prefix = "rcutils-5.1.6",
        url = "https://github.com/ros2/rcutils/archive/refs/tags/5.1.6.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_resource_retriever",
        build_file = "@com_github_mvukov_rules_ros2//repositories:resource_retriever.BUILD.bazel",
        sha256 = "5b4e1411ed955c0562f4609d9025143bf9199d405cbc471484b83f3cbab59162",
        strip_prefix = "resource_retriever-3.1.2",
        url = "https://github.com/ros/resource_retriever/archive/refs/tags/3.1.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rmw",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw.BUILD.bazel",
        sha256 = "fc5eb606c44773a585f6332b33b8fe56c103821cd91e3b95c31a7ab57d38fa0e",
        strip_prefix = "rmw-6.1.2",
        url = "https://github.com/ros2/rmw/archive/refs/tags/6.1.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rmw_cyclonedds",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_cyclonedds.BUILD.bazel",
        sha256 = "58ef4fe3fd18eb723906df77eb10df1e69222b451e479c6ec85426ba48e16a8a",
        strip_prefix = "rmw_cyclonedds-1.3.4",
        url = "https://github.com/ros2/rmw_cyclonedds/archive/refs/tags/1.3.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rmw_dds_common",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_dds_common.BUILD.bazel",
        sha256 = "85dd9f586d53b658e5389a388fe3d99a884ba06f567a59f9908ecb96e29132ef",
        strip_prefix = "rmw_dds_common-1.6.0",
        url = "https://github.com/ros2/rmw_dds_common/archive/refs/tags/1.6.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rmw_implementation",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_implementation.BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rmw_implementation_library_path.patch"],
        sha256 = "c8a4d8160b27aa290eb90f756b6d656011329411a496feab7fb6cf976f964c93",
        strip_prefix = "rmw_implementation-2.8.4",
        url = "https://github.com/ros2/rmw_implementation/archive/refs/tags/2.8.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_robot_state_publisher",
        build_file = "@com_github_mvukov_rules_ros2//repositories:robot_state_publisher.BUILD.bazel",
        sha256 = "74235a379ae3bcaf6a6236ddd36feccea6463749057b09f3409bcbced0c047f9",
        strip_prefix = "robot_state_publisher-3.0.3",
        url = "https://github.com/ros/robot_state_publisher/archive/refs/tags/3.0.3.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_tracing",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_tracing.BUILD.bazel",
        sha256 = "261672e689e583c90b35d97ccea90ffec649ac55a0f045da46cbc3f69b657c5a",
        strip_prefix = "ros2_tracing-4.1.1",
        url = "https://github.com/ros2/ros2_tracing/archive/refs/tags/4.1.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2cli",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2cli.BUILD.bazel",
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2cli_replace-netifaces.patch"],
        sha256 = "b7a1f137839f426fbcbb45727d8cbee9ee60ee9949502e5daf4288513397cefa",
        strip_prefix = "ros2cli-0.18.11",
        url = "https://github.com/ros2/ros2cli/archive/refs/tags/0.18.11.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rosbag2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosbag2.BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rosbag2_relax_plugin_errors.patch"],
        sha256 = "035f4346bdc4bee7b86fed277658bc045b627f5517085fdf3a453285b274ee3c",
        strip_prefix = "rosbag2-0.15.13",
        url = "https://github.com/ros2/rosbag2/archive/refs/tags/0.15.13.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rosidl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl.BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rosidl_rm_unnecessary_asserts.patch"],
        sha256 = "5ff212dd63e3ea99521f323a871641e40aee3f7f896f377a467c19b94e80d01c",
        strip_prefix = "rosidl-3.1.6",
        url = "https://github.com/ros2/rosidl/archive/refs/tags/3.1.6.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rosidl_python",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_python.BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rosidl_python_fix_imports.patch"],
        sha256 = "4bb38b6718a0c23aa6d799548c4cfd021ba320294673e75eaf3137821e1234d1",
        strip_prefix = "rosidl_python-0.14.4",
        url = "https://github.com/ros2/rosidl_python/archive/refs/tags/0.14.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rosidl_runtime_py",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_runtime_py.BUILD.bazel",
        sha256 = "4006ed60e2544eb390a6231c3e7a676d1605601260417b4b207ef94424a38b26",
        strip_prefix = "rosidl_runtime_py-0.9.3",
        url = "https://github.com/ros2/rosidl_runtime_py/archive/refs/tags/0.9.3.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rosidl_typesupport",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_typesupport.BUILD.bazel",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rosidl_typesupport_generate_true_c_code.patch"],
        sha256 = "b330a869ce00eeb5345488fcd4c894464d5a5e3de601c553a9aaad78d2f5b34c",
        strip_prefix = "rosidl_typesupport-2.0.2",
        url = "https://github.com/ros2/rosidl_typesupport/archive/refs/tags/2.0.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rpyutils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rpyutils.BUILD.bazel",
        sha256 = "f87d8c0a2b1a5c28b722f168d7170076e6d82e68c5cb31cff74f15a9fa251fb9",
        strip_prefix = "rpyutils-0.2.1",
        url = "https://github.com/ros2/rpyutils/archive/refs/tags/0.2.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_unique_identifier_msgs",
        build_file = "@com_github_mvukov_rules_ros2//repositories:unique_identifier_msgs.BUILD.bazel",
        sha256 = "ccedcb7c2b6d927fc4f654cceab299a8cb55082953867754795c6ea2ccdd68a9",
        strip_prefix = "unique_identifier_msgs-2.2.1",
        url = "https://github.com/ros2/unique_identifier_msgs/archive/refs/tags/2.2.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_urdfdom",
        build_file = "@com_github_mvukov_rules_ros2//repositories:urdfdom.BUILD.bazel",
        sha256 = "1072b2a304295eb299ed70d99914eb2fbf8843c3257e5e51defc5dd457ee6211",
        strip_prefix = "urdfdom-3.0.2",
        url = "https://github.com/ros/urdfdom/archive/refs/tags/3.0.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_urdfdom_headers",
        build_file = "@com_github_mvukov_rules_ros2//repositories:urdfdom_headers.BUILD.bazel",
        sha256 = "1acd50b976f642de9dc0fde532eb8d77ea09f4de12ebfbd9d88b0cd2891278fd",
        strip_prefix = "urdfdom_headers-1.0.6",
        url = "https://github.com/ros/urdfdom_headers/archive/refs/tags/1.0.6.tar.gz",
    )
