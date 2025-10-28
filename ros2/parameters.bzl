load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def cpp_parameter_library(name, parameter_file, header_name = None):
    if header_name == None:
        header = "{}.h".format(name)
    else:
        header = header_name
    native.genrule(
        name = "{}_generate_cpp".format(name),
        srcs = [
            parameter_file,
        ],
        outs = [header],
        cmd = "./$(location @generate_parameter_library//:generate_cpp_header) $(location {}) $(location {})".format(header, parameter_file),
        tools = [
            "@generate_parameter_library//:generate_cpp_header",
        ],
    )

    cc_library(
        name = name,
        hdrs = [header],
        deps = [
            "@ros2_rclcpp//:rclcpp",
            "@ros2_rclcpp//:rclcpp_lifecycle",
            "@fmt",
            "@rsl",
            "@generate_parameter_library//:parameter_traits",
        ],
    )

def py_parameter_library(name, parameter_file, py_file_name = None):
    if py_file_name == None:
        py = "{}.py".format(name)
    else:
        py = py_file_name
    native.genrule(
        name = "{}_generate_py".format(name),
        srcs = [
            parameter_file,
        ],
        outs = [py],
        cmd = "./$(location @generate_parameter_library//:generate_python_module) $(location {}) $(location {})".format(py, parameter_file),
        tools = [
            "@generate_parameter_library//:generate_python_module",
        ],
    )
    py_library(
        name = name,
        srcs = [py],
        deps = [
            "@ros2_rclpy//:rclpy",
            "@ros2_rcl_interfaces//:py_builtin_interfaces",
            "@generate_parameter_library//:generate_parameter_library_py",
            requirement("pyyaml"),
        ],
    )
