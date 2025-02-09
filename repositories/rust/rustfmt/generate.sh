#!/bin/bash

set -o errexit -o nounset

bazel run @rules_rust//tools/upstream_wrapper:rustfmt --script_path="$(dirname "$0")/rustfmt_wrapper_impl.sh"
