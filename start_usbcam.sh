#!/bin/bash

set -uo pipefail

PIDFILE=/tmp/usb_watchdog.pid
WORKSPACE_DIR=${WORKSPACE_DIR:-$HOME/workspace}
USB_YOLO_CONFIG=${USB_YOLO_CONFIG:-$WORKSPACE_DIR/src/camera_bridge/config/UsbRosConfig.yaml}
USB_YOLO_TOPIC=${USB_YOLO_TOPIC:-}
USB_YOLO_SERIAL_PORT=${USB_YOLO_SERIAL_PORT:-/dev/ttyUSB0}
SERIAL_BAUD=${SERIAL_BAUD:-115200}

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
    echo "[USB-WD][$(date '+%F %T')] $*"
}

##################################################
# ROS环境
##################################################

load_env()
{
    set +u
    source /opt/ros/humble/setup.bash || return 1
    source "$WORKSPACE_DIR/install/setup.bash" || return 1
    set -u
    return 0
}

wait_for_topic()
{
    local topic="$1"
    local timeout="${2:-30}"
    local start_ts
    start_ts=$(date +%s)

    while true
    do
        [ "$STOP_REQUESTED" = "1" ] && return 1

        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            log "Topic ready: $topic"
            return 0
        fi

        if [ $(( $(date +%s) - start_ts )) -ge "$timeout" ]; then
            log "Timeout waiting topic: $topic"
            return 1
        fi

        sleep 0.5
    done
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

    if [ -z "${LAUNCH_PID:-}" ]; then return; fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        LAUNCH_PID=""
        return
    fi

    log "Stopping launch PID=$LAUNCH_PID"

    kill -TERM "$LAUNCH_PID" 2>/dev/null || true

    for i in {1..10}; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
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
# 启动
##################################################

log "======================================"
log "USB Camera Watchdog Started"
log "PID=$$"
log "======================================"

log "Loading ROS environment..."

if ! load_env; then
    log "ROS environment load failed, exit"
    exit 1
fi

log "ROS environment ready"

if [ -z "$USB_YOLO_TOPIC" ] && [ -f "$USB_YOLO_CONFIG" ]; then
    USB_YOLO_TOPIC=$(awk '/^[[:space:]]*topic:/ {print $2; exit}' "$USB_YOLO_CONFIG" 2>/dev/null || true)
fi
USB_YOLO_TOPIC=${USB_YOLO_TOPIC:-/cam_left/image_raw}
log "YOLO camera topic from config: $USB_YOLO_TOPIC"

##################################################
# 主循环
##################################################

while true
do

    [ "$STOP_REQUESTED" = "1" ] && break

    log "Waiting YOLO camera topic: $USB_YOLO_TOPIC"
    if ! wait_for_topic "$USB_YOLO_TOPIC" 30; then
        log "YOLO camera topic not ready, retry..."
        sleep 1
        continue
    fi

    ################################################
    # 启动launch
    ################################################

    LAUNCH_LOG=$(mktemp /tmp/usb_launch_XXXX.log)

    log "Launching usb_detect + serial"

    ros2 launch camera_bridge usb_and_serial.launch.py \
        serial_port:="$USB_YOLO_SERIAL_PORT" \
        serial_baud:="$SERIAL_BAUD" \
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

        # launch 退出
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            wait "$LAUNCH_PID"
            EXIT_CODE=$?
            log "Launch exited code=$EXIT_CODE"
            break
        fi

        # 检测 USB 或 YOLO 错误
        if tail -n 50 "$LAUNCH_LOG" | \
            grep -qiE "(\[error\]|failed to open|camera failed|device failed|engine)"
        then
            log "Error detected, restarting..."
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
