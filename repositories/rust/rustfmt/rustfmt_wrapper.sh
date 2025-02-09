#!/bin/bash

set -o errexit -o nounset

impl=$(dirname "$0")/rustfmt_wrapper_impl.sh

if [ ! -f "${impl}" ]; then
    echo "rustfmt is not set up. Please run ./repositories/rust/rustfmt/generate.sh"
    exit 1
fi

"${impl}" --config-path=`pwd` "$@"
