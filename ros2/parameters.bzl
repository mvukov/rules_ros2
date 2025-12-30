load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def cpp_parameter_library(name, parameter_file, header_name = None,  validate_include_header = "", **kwargs):
    if header_name == None:
        header_name = "{}.h".format(name)

    native.genrule(
        name = "{}_generate_cpp".format(name),
        srcs = [parameter_file],
        outs = [header_name],
        cmd = "./$(location @generate_parameter_library//:generate_cpp_header) $(location {}) $(location {}) {}".format(header_name, parameter_file, validate_include_header),
        tools = [
            "@generate_parameter_library//:generate_cpp_header",
        ],
    )

    cc_library(
        name = name,
        hdrs = [header_name],
        deps = [
            "@ros2_rclcpp//:rclcpp",
            "@ros2_rclcpp//:rclcpp_lifecycle",
            "@fmt",
            "@rsl",
            "@generate_parameter_library//:parameter_traits",
        ],
        **kwargs
    )

def py_parameter_library(name, parameter_file, py_file_name = None, validation_module = "", **kwargs):
    if py_file_name == None:
        py_file_name = "{}.py".format(name)

    native.genrule(
        name = "{}_generate_py".format(name),
        srcs = [parameter_file],
        outs = [py_file_name],
        cmd = "./$(location @generate_parameter_library//:generate_python_module) $(location {}) $(location {}) {}".format(py_file_name, parameter_file, validation_module),
        tools = [
            "@generate_parameter_library//:generate_python_module",
        ],
    )

    py_library(
        name = name,
        srcs = [py_file_name],
        deps = [
            "@ros2_rclpy//:rclpy",
            "@ros2_rcl_interfaces//:py_builtin_interfaces",
            "@generate_parameter_library//:generate_parameter_library_py",
            requirement("pyyaml"),
        ],
        **kwargs
    )
