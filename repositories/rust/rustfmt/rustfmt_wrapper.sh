#!/bin/bash

set -o errexit -o nounset

rustfmt_config=`pwd`/rustfmt.toml

$(dirname "$0")/rustfmt_wrapper_impl.sh --config-path=`pwd` "$@"
