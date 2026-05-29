#!/bin/bash

set -euo pipefail

errors=0

for file in "$@"; do
  basename=$(basename "$file")
  case "$basename" in
    test_*.cc|test_*.h|test_*.py|test_*.rs|test_*.sh)
      echo "$file must not start with 'test_', should end with '_tests.<ext>'"
      errors=$((errors + 1))
      ;;
    *_test.cc|*_test.h|*_test.py|*_test.rs|*_test.sh)
      echo "$file must not end with '_test.<ext>', should end with '_tests.<ext>'"
      errors=$((errors + 1))
      ;;
  esac
done

if [ "$errors" -gt 0 ]; then
  exit 1
fi
