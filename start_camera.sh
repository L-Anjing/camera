#!/bin/bash

set -uo pipefail

PIDFILE=/tmp/camera_watchdog.pid

##################################################
# 状态管理
##################################################

if [ "${1:-}" = "--stop" ]; then

    if [ -f "$PIDFILE" ]; then
        PID=$(cat "$PIDFILE")

        echo "Stopping watchdog PID=$PID"

        kill -TERM "$PID" 2>/dev/null || true

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

##################################################
# PID
##################################################

echo $$ > "$PIDFILE"

##################################################
# 日志
##################################################

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
TAIL_PID=""
STOP_REQUESTED=0

##################################################
# 清理
##################################################

cleanup_launch()
{
    if [ -n "${TAIL_PID:-}" ]; then
        kill "$TAIL_PID" 2>/dev/null || true
        wait "$TAIL_PID" 2>/dev/null || true
        TAIL_PID=""
    fi

    if [ -z "${LAUNCH_PID:-}" ]; then
        return
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        LAUNCH_PID=""
        return
    fi

    log "Stopping launch PID=$LAUNCH_PID"

    # 只杀 launch 进程本身，不影响其他程序
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true

    for i in {1..10}
    do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            break
        fi
        sleep 0.05
    done

    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log "Force killing launch PID=$LAUNCH_PID"
        kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    fi

    LAUNCH_PID=""
}

##################################################
# Ctrl+C
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
# 等待Kinect
##################################################

wait_camera()
{
    while true
    do
        [ "$STOP_REQUESTED" = "1" ] && return 1

        if [ -e /dev/azurekinect ]; then
            return 0
        fi

        sleep 0.05
    done
}

##################################################
# 启动
##################################################

log "======================================"
log "Azure Kinect Watchdog Started"
log "PID=$$"
log "======================================"

log "Loading ROS environment..."

if ! load_env; then
    log "ROS environment load failed, exit"
    exit 1
fi

log "ROS environment ready"

##################################################
# 主循环
##################################################

while true
do

    [ "$STOP_REQUESTED" = "1" ] && break

    ################################################
    # 等待设备
    ################################################

    log "Waiting Azure Kinect..."

    if ! wait_camera; then
        break
    fi

    log "Azure Kinect detected"

    ################################################
    # 启动launch
    ################################################

    LAUNCH_LOG=$(mktemp /tmp/camera_launch_XXXX.log)

    log "Launching camera_bridge"

    ros2 launch camera_bridge k4a_and_serial.launch.py \
        > "$LAUNCH_LOG" 2>&1 &

    LAUNCH_PID=$!

    log "Launch PID=$LAUNCH_PID"

    stdbuf -oL tail -n 0 --pid="$LAUNCH_PID" -f "$LAUNCH_LOG" &
    TAIL_PID=$!

    ################################################
    # 监控
    ################################################

    EXIT_CODE=0

    while true
    do

        [ "$STOP_REQUESTED" = "1" ] && break

        ################################################
        # launch退出
        ################################################

        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then

            wait "$LAUNCH_PID"
            EXIT_CODE=$?

            log "Launch exited code=$EXIT_CODE"

            break
        fi

        ################################################
        # Kinect被拔掉
        ################################################

        if [ ! -e /dev/azurekinect ]; then

            log "Azure Kinect disconnected"

            EXIT_CODE=1

            break
        fi

        ################################################
        # K4A错误
        ################################################

        if tail -n 50 "$LAUNCH_LOG" | \
            grep -qiE "(failed to open|usb command overflow|k4a failed|depth engine|device failed|\[error\])"
        then
            log "K4A error detected"

            EXIT_CODE=1

            break
        fi

        sleep 0.2

    done

    cleanup_launch

    rm -f "$LAUNCH_LOG"

    [ "$STOP_REQUESTED" = "1" ] && break

    log "Restart after 0.1 second..."

    sleep 0.1

done

cleanup_launch

rm -f "$PIDFILE"

log "Watchdog exited"