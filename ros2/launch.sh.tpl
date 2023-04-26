#!{{bash_bin}}

set -o errexit -o nounset -o pipefail

if [ -z "${ROS_HOME:-}" ] && [ -z "${ROS_LOG_DIR:-}" ]; then
  ros_output_dir="${TEST_UNDECLARED_OUTPUTS_DIR:-${TEST_TMPDIR:-}}"
  if [ -n "${ros_output_dir}" ]; then
    export ROS_HOME="${ros_output_dir}"
  fi
fi

ament_prefix_path="{{ament_prefix_path}}"
if [ -z "${ament_prefix_path}" ]; then
  unset AMENT_PREFIX_PATH
  {entry_point} "$@"
else
  AMENT_PREFIX_PATH="${ament_prefix_path}" {entry_point} "$@"
fi
