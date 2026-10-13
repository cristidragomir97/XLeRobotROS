#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/xlerobot_ws/install/setup.bash
if [[ -f /workspace/xlerobot/install/setup.bash ]]; then
    source /workspace/xlerobot/install/setup.bash
fi
exec "$@"
