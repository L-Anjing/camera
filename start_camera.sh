#!/bin/bash

set -uo pipefail

LOG=~/camera_start.log
TERM_TITLE="Azure Kinect Camera Start"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

##################################################
# 新终端启动
##################################################

if [ "${1:-}" != "--run" ]; then

    if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal \
            --title="$TERM_TITLE" \
            -- bash -lc "'$SCRIPT_PATH' --run; exec bash"
        exit 0
    fi

    if [ -n "${DISPLAY:-}" ] && command -v xterm >/dev/null 2>&1; then
        xterm \
            -T "$TERM_TITLE" \
            -e bash -lc "'$SCRIPT_PATH' --run; exec bash" &
        exit 0
    fi

    exec "$SCRIPT_PATH" --run
fi

##################################################
# 日志
##################################################

echo "====================" > "$LOG"
echo "ROS2 Camera Startup" >> "$LOG"
date >> "$LOG"
echo "====================" >> "$LOG"

exec > >(tee -a "$LOG") 2>&1

log()
{
    echo "[$(date '+%F %T')] $*"
}

##################################################
# ROS环境
##################################################

load_env()
{
    set +u

    source /opt/ros/humble/setup.bash || return 1
    source ~/workspace/camera_ws/install/setup.bash || return 1

    set -u
}

##################################################
# 等待设备
##################################################

wait_camera()
{
    while true
    do
        if [ -e /dev/azurekinect ]; then
            return 0
        fi

        sleep 0.2
    done
}

##################################################
# 清理当前Launch
##################################################

LAUNCH_PID=""

cleanup_launch()
{
    if [ -z "${LAUNCH_PID:-}" ]; then
        return
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        LAUNCH_PID=""
        return
    fi

    log "Stopping old camera launch PID=$LAUNCH_PID"

    PGID=$(ps -o pgid= "$LAUNCH_PID" 2>/dev/null | tr -d ' ')

    if [ -n "$PGID" ]; then

        kill -TERM -- "-$PGID" 2>/dev/null || true

        for i in {1..20}
        do
            if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -KILL -- "-$PGID" 2>/dev/null || true
        fi
    fi

    LAUNCH_PID=""
}

##################################################
# Ctrl+C退出时清理
##################################################

trap cleanup_launch EXIT INT TERM

##################################################
# 主循环
##################################################

log "Camera watchdog started"

while true
do

    ################################################
    # 环境检查
    ################################################

    if ! load_env; then
        log "ROS environment load failed"

        sleep 1
        continue
    fi

    ################################################
    # 等待设备
    ################################################

    wait_camera

    log "Azure Kinect detected"

    ################################################
    # 启动Launch
    ################################################

    log "Launching camera_bridge..."

    ros2 launch camera_bridge k4a_and_serial.launch.py &
    LAUNCH_PID=$!

    log "Launch PID=$LAUNCH_PID"

    ################################################
    # 等待退出
    ################################################

    wait "$LAUNCH_PID"
    EXIT_CODE=$?

    log "Launch exited code=$EXIT_CODE"

    ################################################
    # 清理残留
    ################################################

    cleanup_launch

    ################################################
    # 快速重试
    ################################################

    sleep 1

done