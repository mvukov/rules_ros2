load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")

def cpp_parameter_library(name, parameter_file, header_name = None):
    if cpp_header_name == None:
        header = "{}.hpp".format(name)
    else:
        header = cpp_header_name
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
        includes = ["."],
        deps = [
            "@ros2_rclcpp//:rclcpp",
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
            "@ros2_rcl_interfaces//:py_builtin_interfaces"
            requirement("pyyaml"),
        ],
    )
