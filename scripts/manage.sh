#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
export BUILDX_BUILDER="${BUILDX_BUILDER:-default}"
compose=(docker compose -f "$ROOT/docker-compose.yml")
if [[ -c /dev/dxg && -f /usr/lib/wsl/lib/libd3d12.so ]]; then
    compose+=(-f "$ROOT/docker-compose.wsl.yml")
elif [[ -d /dev/dri ]]; then
    compose+=(-f "$ROOT/docker-compose.dri.yml")
fi
inside() {
    local tty=()
    [[ -t 0 && -t 1 ]] || tty=(-T)
    "${compose[@]}" exec "${tty[@]}" ros-dev /usr/local/bin/xlerobot-entrypoint "$@"
}
stop_sim() {
    if [[ -n "$("${compose[@]}" ps --status running -q ros-dev)" ]]; then
        inside python3 scripts/stop_sim.py
    fi
}
usage() {
    cat <<'HELP'
Usage: ./scripts/manage.sh <command> [arguments]
  start                 Initialize upstream, build image, start development container.
  stop                  Stop this project's containers.
  sim-stop              Stop simulation gracefully; keep the development container running.
  restart               Restart this project's containers.
  recreate              Build and recreate container, preserve volumes.
  logs                  Follow container logs.
  exec [command...]     Run a sourced command or shell in the container.
  build [package...]    Build workspace source after edits.
  sim [standard|mod101] [launch args...]  MuJoCo + RGB-D + IMU + RTAB-Map + dashboard (localhost:8889).
  robocore_viz [standard|mod101] [args...]  Run the configured ROS dashboard (localhost:8889).
  teleop                Keyboard base control (including lateral movement).
  rviz                  Robot, RGB-D cloud and map in RViz.
  world-assets          Prepare attributed RoboCasa home furniture and objects.
  test [standard|mod101]  Check description/physics/sensors headlessly.
  test --sim            Verify running ROS sensors, TF, SLAM and robot actions.
  test --dashboard      Verify dashboard sensors, actions, strafe and watchdog.
  test --tracking       Exercise visual tracking loss and automatic recovery.
  test --clearance [--sweep]  Check actual Mod101/tower mesh clearance; optional range study.
  test --nav            Send a short navigation goal to a running nav:=true simulation.
HELP
}
action=${1:-help}; (( $# == 0 )) || shift
case "$action" in
    start|recreate)
        git submodule update --init upstream dependencies/lekiwi_ros2 dependencies/mod101
        git -C dependencies/lekiwi_ros2 submodule update --init modules/sts_hardware_interface
        bash scripts/setup-world-assets.sh
        "${compose[@]}" build ros-dev
        stop_sim
        options=(-d); [[ "$action" != recreate ]] || options+=(--force-recreate)
        "${compose[@]}" up "${options[@]}" ros-dev
        printf 'Ready. Run ./scripts/manage.sh sim\n'
        ;;
    sim-stop) stop_sim ;;
    stop) stop_sim; "${compose[@]}" stop ;;
    restart) stop_sim; "${compose[@]}" restart ;;
    logs) "${compose[@]}" logs -f ;;
    exec) (( $# != 0 )) || set -- bash; inside "$@" ;;
    build)
        select=(--packages-select lekiwi_description lekiwi_mujoco lekiwi_control sts_hardware_interface mod101_description mod101_tool_parallel mod101_tool_pggripper mod101_tool_pincopen mod101_tool_jaws mod101_tool_none mod101_tool_camera robocore_viz xlerobot_description xlerobot_control xlerobot_sim); (( $# == 0 )) || select=(--packages-select "$@")
        inside colcon build --symlink-install --base-paths src dependencies/lekiwi_ros2/lekiwi_description \
            dependencies/lekiwi_ros2/lekiwi_mujoco dependencies/lekiwi_ros2/lekiwi_control \
            dependencies/lekiwi_ros2/modules/sts_hardware_interface dependencies/mod101/src/mod101_description dependencies/mod101/src/mod101_tool_parallel dependencies/mod101/src/mod101_tool_pggripper dependencies/mod101/src/mod101_tool_pincopen dependencies/mod101/src/mod101_tool_jaws dependencies/mod101/src/mod101_tool_none dependencies/mod101/src/mod101_tool_camera --parallel-workers 2 \
            "${select[@]}" --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF ;;
    sim)
        configuration=${XLEROBOT_CONFIGURATION:-standard}
        case "${1:-}" in standard|mod101) configuration=$1; shift ;; esac
        case "$configuration" in standard|mod101) ;; *) printf 'Unknown configuration: %s\n' "$configuration" >&2; exit 2 ;; esac
        gui=true
        [[ "${XLEROBOT_HEADLESS:-false}" == false ]] || gui=false
        for arg in "$@"; do
            case "$arg" in headless:=false) gui=true ;; headless:=true) gui=false ;; esac
        done
        if [[ "$gui" == true ]] && command -v xhost >/dev/null; then
            xhost +si:localuser:root >/dev/null
        fi
        inside bash scripts/run_sim.sh "$configuration" "headless:=${XLEROBOT_HEADLESS:-false}" "$@" ;;
    world-assets) bash scripts/setup-world-assets.sh ;;
    robocore_viz)
        configuration=${XLEROBOT_CONFIGURATION:-standard}
        case "${1:-}" in standard|mod101) configuration=$1; shift ;; esac
        inside python3 -m xlerobot_sim.dashboard "$configuration" "$@" ;;
    teleop) inside ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.4 ;;
    rviz) inside ros2 run rviz2 rviz2 -d /workspace/xlerobot/src/xlerobot_sim/config/xlerobot.rviz --ros-args -p use_sim_time:=true ;;
    test)
        if [[ "${1:-}" == --sim ]]; then
            inside python3 checks/check_control_stack.py
            inside python3 checks/check_runtime.py
        elif [[ "${1:-}" == --tracking ]]; then
            inside python3 checks/check_tracking_recovery.py
        elif [[ "${1:-}" == --dashboard ]]; then
            inside python3 checks/check_dashboard.py
        elif [[ "${1:-}" == --clearance ]]; then
            shift
            inside python3 checks/check_tower_clearance.py "$@"
        elif [[ "${1:-}" == --nav ]]; then
            inside python3 checks/check_navigation.py
        else
            inside python3 checks/check_model.py --configuration "${1:-${XLEROBOT_CONFIGURATION:-standard}}"
        fi ;;
    help|-h|--help) usage ;;
    *) usage >&2; exit 2 ;;
esac
