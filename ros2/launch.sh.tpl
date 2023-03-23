#!{{bash_bin}}

set -o errexit -o nounset -o pipefail

ament_prefix_path="{{ament_prefix_path}}"
if [ -z "${ament_prefix_path}" ]; then
  unset AMENT_PREFIX_PATH
  {entry_point} "$@"
else
  AMENT_PREFIX_PATH="${ament_prefix_path}" {entry_point} "$@"
fi
