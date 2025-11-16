#!/bin/bash

set -o errexit -o nounset -o pipefail

repository_name=`grep '^,' {{binary_path}}.runfiles/_repo_mapping | cut -d',' -f2`
cd {{binary_path}}.runfiles/${repository_name}
exec {{binary_path}} "$@"
