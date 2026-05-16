#!/bin/bash
# Kick runner — run ON Dog.
#
# Usage:
#   ./run_kick_on_dog.sh --goal goal_right                        # build + run
#   ./run_kick_on_dog.sh --goal goal_left  --no-build             # skip build
#   ./run_kick_on_dog.sh --goal goal_right --server 10.0.0.252    # custom VRPN

set -euo pipefail
trap 'code=$?; echo "[ERROR] failed at line $LINENO (exit $code)" >&2; exit $code' ERR

VRPN_SERVER="10.0.0.252"
DO_BUILD=1
GOAL_TRACKER=""

usage() {
    echo "Usage: $0 --goal <goal_right|goal_left> [--server <ip>] [--build|--no-build]"
    echo ""
    echo "Required:"
    echo "  --goal GOAL    goal_right or goal_left"
    echo ""
    echo "Optional:"
    echo "  --server IP    VRPN server IP (default: 10.0.0.252)"
    echo "  --build        force build (default)"
    echo "  --no-build     skip colcon build"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --goal)      GOAL_TRACKER="$2"; shift 2 ;;
        --server)    VRPN_SERVER="$2";   shift 2 ;;
        --build)     DO_BUILD=1;         shift   ;;
        --no-build)  DO_BUILD=0;         shift   ;;
        -h|--help)   usage ;;
        *) echo "Unknown: $1"; usage ;;
    esac
done

if [[ "$GOAL_TRACKER" != "goal_right" && "$GOAL_TRACKER" != "goal_left" ]]; then
    echo "ERROR: --goal is required and must be 'goal_right' or 'goal_left'"
    usage
fi

source /etc/mi/ros2_env.conf

set +u
source ~/vrpn_client_ros2/src/install/local_setup.bash 2>/dev/null || true
source ~/ros2_ws/install/local_setup.bash 2>/dev/null || true
set -u

echo "========================================="
echo "  Kick Runner (on Dog)"
echo "========================================="
echo "  Goal:        $GOAL_TRACKER"
echo "  VRPN server: $VRPN_SERVER"
echo "  Build:       $([[ $DO_BUILD -eq 1 ]] && echo YES || echo SKIP)"
echo "========================================="

# ── Step 1: Build ────────────────────────────────────────────
if [[ "$DO_BUILD" -eq 1 ]]; then
    echo "[1/3] Building demo_python_pkg ..."
    cd ~/ros2_ws
    colcon build --packages-select demo_python_pkg
    echo "       Build done."
else
    echo "[1/3] Build skipped (using cached install)."
fi

# ── Step 2: Start VRPN listener ──────────────────────────────
echo "[2/3] Starting VRPN listener ..."
sed -i "s/server: .*/server: $VRPN_SERVER/" ~/vrpn_client_ros2/src/vrpn_listener/config/params.yaml

if pgrep -f vrpn_listener/lib/vrpn_listener >/dev/null 2>&1; then
    echo "       VRPN listener already running (pid=$(pgrep -f vrpn_listener/lib/vrpn_listener))"
else
    nohup ros2 run vrpn_listener vrpn_listener --ros-args \
        --params-file ~/vrpn_client_ros2/src/vrpn_listener/config/params.yaml \
        >/tmp/vrpn.log 2>&1 &
    sleep 2
    echo "       VRPN listener started (pid=$!)"
fi

# ── Step 3: Run kicker ───────────────────────────────────────
echo "[3/3] Running kicker (Ctrl+C to stop) ..."
echo ""
ros2 run demo_python_pkg kick --ros-args -p goal_tracker:="$GOAL_TRACKER"
