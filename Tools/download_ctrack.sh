#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

DEST_PATH="$SCRIPT_DIR/simulation/gazebo-classic/sitl_gazebo-classic/models/c-track/meshes"
if [ ! -d "$DEST_PATH" ]; then
  mkdir "$DEST_PATH"
fi

python3 -m gdown --continue 13TmnltIcraQ770KEB1bI-RRVQ1CgvL_1 -O "$DEST_PATH/c-track.stl"
