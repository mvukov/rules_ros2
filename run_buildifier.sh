#!/bin/bash

cd "$(git rev-parse --show-toplevel)"
buildifier -lint=warn WORKSPACE $(find . -type f -name "*.bazel") $(find . -type f -name "*.bzl")
buildifier -lint=fix WORKSPACE $(find . -type f -name "*.bazel") $(find . -type f -name "*.bzl")
