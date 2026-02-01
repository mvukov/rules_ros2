load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_python//python:defs.bzl", "py_library")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def cpp_parameter_library(name, parameter_file, header_name = None, validate_include_header = None, **kwargs):
    if header_name == None:
        header_name = "{}.h".format(name)

    # Build the validate header argument for the genrule command
    # We need to pass the package-relative path for the C++ #include directive
    validate_include_path = ""
    if validate_include_header != None:
        # Get the package name for constructing the full include path
        package = native.package_name()

        # Extract filename from label (handle :file.hpp or just file.hpp)
        validate_file = validate_include_header.split(":")[-1] if ":" in validate_include_header else validate_include_header

        # Construct the full package-relative path
        validate_include_path = "{}/{}".format(package, validate_file) if package else validate_file

    # Prepare sources and hdrs lists
    srcs = [parameter_file]
    hdrs = [header_name]
    if validate_include_header != None:
        srcs.append(validate_include_header)
        hdrs.append(validate_include_header)

    native.genrule(
        name = "{}_generate_cpp".format(name),
        srcs = srcs,
        outs = [header_name],
        cmd = "./$(location @generate_parameter_library//:generate_cpp_header) $(location {}) $(location {}) {}".format(header_name, parameter_file, validate_include_path),
        tools = [
            "@generate_parameter_library//:generate_cpp_header",
        ],
    )

    cc_library(
        name = name,
        hdrs = hdrs,
        deps = [
            "@ros2_rclcpp//:rclcpp",
            "@ros2_rclcpp//:rclcpp_lifecycle",
            "@fmt",
            "@rsl",
            "@generate_parameter_library//:parameter_traits",
        ],
        **kwargs
    )

def py_parameter_library(name, parameter_file, py_file_name = None, validation_module = None, **kwargs):
    if py_file_name == None:
        py_file_name = "{}.py".format(name)

    # Build the validation module path for Python imports
    # We need to convert the file path to a Python module path (e.g., ros2/test/foo.py -> ros2.test.foo)
    validation_module_path = ""
    if validation_module != None:
        # Get the package name for constructing the full module path
        package = native.package_name()

        # Extract filename from label (handle :file.py or just file.py)
        validation_file = validation_module.split(":")[-1] if ":" in validation_module else validation_module

        # Remove .py extension if present
        validation_file_base = validation_file[:-3] if validation_file.endswith(".py") else validation_file

        # Construct the full module path with dots (e.g., ros2.test.generate_parameter.test_validators)
        validation_module_path = "{}.{}".format(package.replace("/", "."), validation_file_base) if package else validation_file_base

    # Prepare sources list
    srcs = [parameter_file]
    py_srcs = [py_file_name]
    if validation_module != None:
        srcs.append(validation_module)
        py_srcs.append(validation_module)

    native.genrule(
        name = "{}_generate_py".format(name),
        srcs = srcs,
        outs = [py_file_name],
        cmd = "./$(location @generate_parameter_library//:generate_python_module) $(location {}) $(location {}) {}".format(py_file_name, parameter_file, validation_module_path),
        tools = [
            "@generate_parameter_library//:generate_python_module",
        ],
    )

    py_library(
        name = name,
        srcs = py_srcs,
        deps = [
            "@ros2_rclpy//:rclpy",
            "@ros2_rcl_interfaces//:py_builtin_interfaces",
            "@generate_parameter_library//:generate_parameter_library_py",
            requirement("pyyaml"),
        ],
        **kwargs
    )
