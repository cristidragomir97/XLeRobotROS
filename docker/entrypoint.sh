#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/xlerobot_ws/install/setup.bash
# A development volume can contain a copied native build with host-only paths.
# Only source a usable container overlay, without replaying its parent chain.
if [[ -f /workspace/xlerobot/install/local_setup.bash ]]; then
    source /workspace/xlerobot/install/local_setup.bash
fi
# Use the bind-mounted source so edits apply without rebuilding the image.
if [[ -d /workspace/xlerobot/src/xlerobot_sim ]]; then
    export PYTHONPATH="/workspace/xlerobot/src/xlerobot_sim:$PYTHONPATH"
fi
exec "$@"
