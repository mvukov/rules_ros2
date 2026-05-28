import os
import sys

env = os.environ.copy()

if env.get("BAZEL_TEST"):
    bazel_test_output_dir = env.get(
        "TEST_UNDECLARED_OUTPUTS_DIR", env.get("TEST_TMPDIR", "")
    )
    env.setdefault("ROS_HOME", bazel_test_output_dir)
    env.setdefault("ROS_LOG_DIR", bazel_test_output_dir)

ament_prefix_path = "{{ament_prefix_path}}"
if not ament_prefix_path:
    env.pop("AMENT_PREFIX_PATH", None)
else:
    env["AMENT_PREFIX_PATH"] = ament_prefix_path

entry_point = "{entry_point}"
os.execve(entry_point, [entry_point, *sys.argv[1:]], env)
