#!/bin/bash

if [[ ! -z "${BUILDBUDDY_ORG_API_KEY}" ]]; then
  BAZEL_REMOTE_CONFIG=(
    --config=remote
    --remote_header=x-buildbuddy-api-key="${BUILDBUDDY_ORG_API_KEY}"
  )
fi
bazel --bazelrc=$GITHUB_WORKSPACE/.github/workflows/ci.bazelrc --bazelrc=.bazelrc \
    test "${BAZEL_REMOTE_CONFIG[@]}" //... "$@"

bazel --bazelrc=$GITHUB_WORKSPACE/.github/workflows/ci.bazelrc --bazelrc=.bazelrc \
    build "${BAZEL_REMOTE_CONFIG[@]}" $(bazel query '//external:*' | grep ":ros2_" | sed 's#//external:\(ros2_.*\)#\@\1//...#') "$@"
