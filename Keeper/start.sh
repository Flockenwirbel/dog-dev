#!/bin/bash
# ============================================
#  机器狗大赛启动脚本 — Keeper (守门员)
#  位置: /home/mi/start.sh
#  用途: 从控制电脑 SSH 远程启动
#
#  用法:
#    ./start.sh dog2 goal_right
#    ./start.sh dog2 goal_left
# ============================================

DOG=${1:-}
GOAL=${2:-}

if [[ "${DOG}" != "dog2" ]]; then
    echo "ERROR: this start.sh is for Keeper (dog2), got '${DOG}'"
    echo "Usage: $0 dog2 <goal_right|goal_left>"
    exit 1
fi

if [[ "${GOAL}" != "goal_right" && "${GOAL}" != "goal_left" ]]; then
    echo "ERROR: goal must be 'goal_right' or 'goal_left', got '${GOAL}'"
    echo "Usage: $0 dog2 <goal_right|goal_left>"
    exit 1
fi

echo ""
echo "========================================"
echo "  MACHINE DOG MATCH — KEEPER"
echo "  Dog:  ${DOG}"
echo "  Goal: ${GOAL}"
echo "  Time: $(date)"
echo "========================================"
echo ""

# ============================================
# 1. 加载环境
# ============================================
echo "[1/2] Loading environment..."

source /opt/ros2/galactic/setup.bash 2>/dev/null
source /opt/ros2/cyberdog/local_setup.bash 2>/dev/null
source /home/mi/vrpn_client_ros2/src/install/setup.bash 2>/dev/null

# Keeper 工作空间
source /home/mi/goalkeeper_ws/install/setup.bash 2>/dev/null

# ============================================
# 2. 启动守门员
# ============================================
echo "[2/2] Starting goalkeeper ..."
echo ""
echo "========================================"
echo "  ${DOG} RUNNING (${GOAL})"
echo "  Press Ctrl+C to stop"
echo "========================================"
echo ""

DOG_NAME=${DOG} ros2 run goalkeeper_pkg gk --ros-args -p goal_tracker:=${GOAL}
