#!/bin/sh

set -e # Add pipe fail command to exit on error

# Masking command outputs
mask() {
  echo "[$(date +'%Y-%m-%dT%H:%M:%S%z')] [MASKED] $@"
}

echo "=====< :: Setting Up :: >====="
echo ""
echo " :: Adding Developer to .cache :: "
sudo chown developer:developer ~/.cache
export PATH="/home/developer/.local/bin:$PATH"
echo ""
echo " :: Setting the CC Variable :: "
export CC=$(which clang-17) >> ~/.bashrc
echo ""
echo " :: Done :: "

# Add any additional pre-configuration steps
echo "=====< :: Exiting :: >====="
