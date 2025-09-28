load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")

def parameter_library(name, parameter_file, cpp_header_name = None, python_file_name = None):
    if cpp_header_name == None:
        header = "{}.hpp".format(name)
    else:
        header = cpp_header_name
    if python_file_name == None:
        py = "{}.py".format(name)
    else:
        py = python_file_name
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
        name = "{}_cpp".format(name),
        hdrs = [header],
        includes = ["."],
        deps = [
            "@ros2_rclcpp//:rclcpp",
            "@fmt",
            "@rsl",
            "@generate_parameter_library//:parameter_traits",
        ],
    )
