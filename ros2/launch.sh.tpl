#!{{bash_bin}}

set -o errexit -o nounset -o pipefail

if [ -n "{{set_up_ros_home}}" ]; then
  if [ -z "${ROS_HOME:-}" ] && [ -z "${ROS_LOG_DIR:-}" ]; then
    ros_output_dir="${TEST_UNDECLARED_OUTPUTS_DIR:-${TEST_TMPDIR:-}}"
    if [ -n "${ros_output_dir}" ]; then
      export ROS_HOME="${ros_output_dir}"
      export ROS_LOG_DIR="${ros_output_dir}"
    fi
  fi
fi

if [ -n "{{set_up_ament}}" ]; then
  unset AMENT_PREFIX_PATH
  if [ -n "{{ament_prefix_path}}" ]; then
    export AMENT_PREFIX_PATH="{{ament_prefix_path}}"
  fi
fi

"{{entry_point}}" "$@"
