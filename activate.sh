# Source this once per terminal to work in this project:
#   source ~/Vegam/activate.sh
#!/usr/bin/env bash
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

export LIBGL_ALWAYS_SOFTWARE=1
export PYTHONPATH=$HOME/Vegam:$PYTHONPATH
echo "[Vegam] workspace active: $WS_DIR"
