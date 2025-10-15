#!{{sh_bin}}

set -eu

if [ -n "${BAZEL_TEST:-}" ]; then
  bazel_test_output_dir="${TEST_UNDECLARED_OUTPUTS_DIR:-${TEST_TMPDIR}}"
  if [ -z "${ROS_HOME:-}" ]; then
    export ROS_HOME="${bazel_test_output_dir}"
  fi
  if [ -z "${ROS_LOG_DIR:-}" ]; then
    export ROS_LOG_DIR="${bazel_test_output_dir}"
  fi
fi

ament_prefix_path="{{ament_prefix_path}}"
if [ -z "${ament_prefix_path}" ]; then
  unset AMENT_PREFIX_PATH
  exec {entry_point} "$@"
else
  AMENT_PREFIX_PATH="${ament_prefix_path}" exec {entry_point} "$@"
fi
