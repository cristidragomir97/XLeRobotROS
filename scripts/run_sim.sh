#!/usr/bin/env bash
set -euo pipefail
configuration=${1:-standard}
(( $# == 0 )) || shift
case "$configuration" in
    standard|mod101) launch_file="sim_${configuration}.launch.py" ;;
    *) printf 'Unknown configuration: %s\n' "$configuration" >&2; exit 2 ;;
esac
# flock releases automatically even after interruption; only one physics/TF owner per runtime.
mkdir -p .runtime
exec 9>.runtime/simulator.lock
flock -n 9 || { printf 'An XLeRobot simulation is already running.\n' >&2; exit 1; }
exec ros2 launch xlerobot_sim "$launch_file" "$@"
