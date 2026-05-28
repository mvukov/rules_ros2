#!/bin/bash

set -euo pipefail

errors=0

for file in "$@"; do
  case "$file" in
    *.hpp|*.hxx|*.hh|*.H)
      echo "ERROR: C++ header file must use .h extension: $file"
      errors=$((errors + 1))
      ;;
    *.cpp|*.cxx|*.c++|*.C)
      echo "ERROR: C++ source file must use .cc extension: $file"
      errors=$((errors + 1))
      ;;
  esac
done

if [ "$errors" -gt 0 ]; then
  exit 1
fi
