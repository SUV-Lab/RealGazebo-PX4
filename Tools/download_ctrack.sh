#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

python3 -m gdown --continue 13TmnltIcraQ770KEB1bI-RRVQ1CgvL_1 -O $SCRIPT_DIR/simulation/gazebo-classic/sitl_gazebo-classic/models/c-track/meshes/c-track.stl