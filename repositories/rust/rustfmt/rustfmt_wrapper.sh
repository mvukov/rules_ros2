#!/bin/bash

set -o errexit -o nounset

$(dirname "$0")/rustfmt_wrapper_impl.sh --config-path=`pwd` "$@"
