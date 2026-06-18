#!/bin/bash

set -uo pipefail

LOG=~/camera_start.log
PIDFILE=/tmp/camera_watchdog.pid

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

##################################################
# 打开独立终端
##################################################

if [ "${1:-}" != "--run" ]; then

    if [ "${1:-}" = "--stop" ]; then

        if [ -f "$PIDFILE" ]; then
            PID=$(cat "$PIDFILE")

            echo "Stopping watchdog PID=$PID"

            kill -TERM "$PID" 2>/dev/null

            rm -f "$PIDFILE"
        else
            echo "Watchdog not running"
        fi

        exit 0
    fi

    if [ "${1:-}" = "--status" ]; then

        if [ -f "$PIDFILE" ]; then

            PID=$(cat "$PIDFILE")

            if kill -0 "$PID" 2>/dev/null; then
                echo "Running PID=$PID"
            else
                echo "PID file exists but process dead"
            fi

        else
            echo "Not running"
        fi

        exit 0
    fi

    if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal \
            --title="Azure Kinect Watchdog" \
            -- bash -lc "'$SCRIPT_PATH' --run; exec bash"
        exit 0
    fi

    exec "$SCRIPT_PATH" --run
fi

##################################################
# 日志
##################################################

echo $$ > "$PIDFILE"

exec > >(tee -a "$LOG") 2>&1

log()
{
    echo "[CAMERA-WD][$(date '+%F %T')] $*"
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

    return 0
}

##################################################
# 全局变量
##################################################

LAUNCH_PID=""
STOP_REQUESTED=0

##################################################
# 清理当前launch
##################################################

cleanup_launch()
{
    if [ -z "${LAUNCH_PID:-}" ]; then
        return
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        LAUNCH_PID=""
        return
    fi

    log "Stopping camera launch PID=$LAUNCH_PID"

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
# Ctrl+C处理
##################################################

shutdown()
{
    log "User requested stop"

    STOP_REQUESTED=1

    cleanup_launch

    rm -f "$PIDFILE"

    exit 0
}

trap shutdown SIGINT SIGTERM

##################################################
# 等待设备
##################################################

wait_camera()
{
    while true
    do

        if [ "$STOP_REQUESTED" = "1" ]; then
            return 1
        fi

        if [ -e /dev/azurekinect ]; then
            return 0
        fi

        sleep 0.2
    done
}

##################################################
# 启动信息
##################################################

log "========================================="
log "Azure Kinect Watchdog Started"
log "PID=$$"
log "Log file: $LOG"
log "Stop command:"
log "kill $$"
log "========================================="

##################################################
# 主循环
##################################################

while true
do

    if [ "$STOP_REQUESTED" = "1" ]; then
        break
    fi

    ################################################
    # ROS环境
    ################################################

    if ! load_env; then
        log "ROS environment load failed"

        sleep 1
        continue
    fi

    ################################################
    # 等待设备
    ################################################

    log "Waiting Azure Kinect..."

    if ! wait_camera; then
        break
    fi

    log "Azure Kinect detected"

    ################################################
    # 启动launch + 日志监控
    ################################################

    LAUNCH_LOG=$(mktemp /tmp/camera_launch_XXXX.log)

    log "Launching camera_bridge... (log=$LAUNCH_LOG)"

    ros2 launch camera_bridge k4a_and_serial.launch.py \
        > "$LAUNCH_LOG" 2>&1 &
    LAUNCH_PID=$!

    log "Launch PID=$LAUNCH_PID"

    ################################################
    # 实时监控: K4A open error → 立即重启
    ################################################

    EXIT_CODE=0
    while true
    do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            wait "$LAUNCH_PID"
            EXIT_CODE=$?
            break
        fi

        # 检测 K4A open 错误 (来源: camera_k4a.cpp Open())
        if grep -q "Open K4a Device Error\|Failed to open K4a device" \
            "$LAUNCH_LOG" 2>/dev/null
        then
            log "K4A open error detected, restarting..."
            cleanup_launch
            EXIT_CODE=1
            break
        fi

        sleep 0.5
    done

    rm -f "$LAUNCH_LOG"

    ################################################
    # 如果是用户Ctrl+C
    ################################################

    if [ "$STOP_REQUESTED" = "1" ]; then
        break
    fi

    ################################################
    # launch异常退出
    ################################################

    log "Launch exited code=$EXIT_CODE"

    cleanup_launch

    log "Restart after 1 second..."

    sleep 1

done

cleanup_launch

rm -f "$PIDFILE"

log "Watchdog exited"