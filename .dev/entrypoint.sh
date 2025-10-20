#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash || true
export AMENT_PREFIX_PATH="/ws/install:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="/ws/install:${CMAKE_PREFIX_PATH:-}"
fi
exec "$@"
