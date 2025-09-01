#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Find git repository root and change to it
# First try to get the superproject root (in case we're in a submodule)
GIT_ROOT=$(git rev-parse --show-superproject-working-tree 2>/dev/null)
if [ -z "$GIT_ROOT" ]; then
    # If not in a submodule, get the regular git root
    GIT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)
fi

if [ -z "$GIT_ROOT" ]; then
    echo "Error: Not in a git repository"
    exit 1
fi

cd "$GIT_ROOT"

if git submodule status Tools/simulation/gz | grep -q '^-'; then
      echo "initializing submodule"
      git submodule update --init --recursive Tools/simulation/gz
fi

DEST_PATH="$GIT_ROOT/Tools/simulation/gz/models/c-track/meshes"
if [ ! -d "$DEST_PATH" ]; then
  mkdir "$DEST_PATH"
fi

wget -q https://realgazebo.cbnu.ac.kr/file/c-track.stl -O "$DEST_PATH/c-track.stl"