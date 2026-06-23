#!{{bash_bin}}

set -o errexit -o nounset -o pipefail

exec {{ground_control_bin}} {{ground_control_config}}
