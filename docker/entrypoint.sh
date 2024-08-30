#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
source "/home/colcon_ws/install/setup.bash"

exec "$@"