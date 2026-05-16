#!/bin/bash
# ============================================
#  机器狗大赛启动脚本 — Goaler (前锋)
#  位置: /home/mi/start.sh
#  用途: 从控制电脑 SSH 远程启动
#
#  用法:
#    ./start.sh dog1 goal_right
#    ./start.sh dog1 goal_left
# ============================================

DOG=${1:-}
GOAL=${2:-}

if [[ "${DOG}" != "dog1" ]]; then
    echo "ERROR: this start.sh is for Goaler (dog1), got '${DOG}'"
    echo "Usage: $0 dog1 <goal_right|goal_left>"
    exit 1
fi

if [[ "${GOAL}" != "goal_right" && "${GOAL}" != "goal_left" ]]; then
    echo "ERROR: goal must be 'goal_right' or 'goal_left', got '${GOAL}'"
    echo "Usage: $0 dog1 <goal_right|goal_left>"
    exit 1
fi

echo ""
echo "========================================"
echo "  MACHINE DOG MATCH — GOALER"
echo "  Dog:  ${DOG}"
echo "  Goal: ${GOAL}"
echo "  Time: $(date)"
echo "========================================"
echo ""

# ── 1. 加载环境 ────────────────────────────
echo "[1/2] Loading environment..."

source /opt/ros2/galactic/setup.bash 2>/dev/null
source /opt/ros2/cyberdog/local_setup.bash 2>/dev/null
source /home/mi/vrpn_client_ros2/src/install/setup.bash 2>/dev/null

# 策略代码工作空间
source /home/mi/ros2_ws/install/setup.bash 2>/dev/null

# ── 2. 启动 ────────────────────────────────
echo "[2/2] Starting striker ..."
echo ""
echo "========================================"
echo "  ${DOG} RUNNING (${GOAL})"
echo "  Press Ctrl+C to stop"
echo "========================================"
echo ""

DOG_NAME=${DOG} ros2 run demo_python_pkg striker --ros-args -p goal_tracker:=${GOAL}
