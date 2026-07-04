#!/bin/bash

set -uo pipefail

PIDFILE=/tmp/orbbec_watchdog.pid
WORKSPACE_DIR=${WORKSPACE_DIR:-$HOME/workspace/camera_ws}
ORBBEC_SERIAL_PORT=${ORBBEC_SERIAL_PORT:-/dev/azurekinect}
SERIAL_BAUD=${SERIAL_BAUD:-115200}

log() { echo "[ORBBEC-WD][$(date '+%F %T')] $*"; }

cleanup_launch() {
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

    for _ in {1..20}; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
        sleep 0.05
    done

    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log "Force killing launch PID=$LAUNCH_PID"
        kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    fi

    LAUNCH_PID=""
}

stop_watchdog() {
    if [ -f "$PIDFILE" ]; then
        PID=$(cat "$PIDFILE")
        echo "Stopping watchdog PID=$PID"
        kill -TERM "$PID" 2>/dev/null || true
        rm -f "$PIDFILE"
    else
        echo "Watchdog not running"
    fi
}

status_watchdog() {
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
}

if [ "${1:-}" = "--stop" ]; then
    stop_watchdog
    exit 0
fi

if [ "${1:-}" = "--status" ]; then
    status_watchdog
    exit 0
fi

if [ -f "$PIDFILE" ]; then
    OLD_PID=$(cat "$PIDFILE")
    if kill -0 "$OLD_PID" 2>/dev/null; then
        echo "Watchdog already running PID=$OLD_PID"
        exit 0
    fi
    rm -f "$PIDFILE"
fi

echo $$ > "$PIDFILE"

LAUNCH_PID=""
TAIL_PID=""
STOP_REQUESTED=0

shutdown() {
    log "Stop requested"
    STOP_REQUESTED=1
    cleanup_launch
    rm -f "$PIDFILE"
    exit 0
}

trap shutdown SIGINT SIGTERM

load_env() {
    set +u
    source /opt/ros/humble/setup.bash || return 1
    source "$WORKSPACE_DIR/install/setup.bash" || return 1
    set -u
    return 0
}

log "======================================"
log "Orbbec Watchdog Started"
log "PID=$$"
log "======================================"

log "Loading ROS environment..."
if ! load_env; then
    log "ROS environment load failed, exit"
    rm -f "$PIDFILE"
    exit 1
fi
log "ROS environment ready"

while true; do
    [ "$STOP_REQUESTED" = "1" ] && break

    LAUNCH_LOG=$(mktemp /tmp/orbbec_launch_XXXX.log)
    log "Launching orbbec_detect + serial"

    ros2 launch camera_bridge orbbec_and_serial.launch.py \
        serial_port:="$ORBBEC_SERIAL_PORT" \
        serial_baud:="$SERIAL_BAUD" \
        > "$LAUNCH_LOG" 2>&1 &

    LAUNCH_PID=$!
    log "Launch PID=$LAUNCH_PID"

    stdbuf -oL tail -n 0 --pid="$LAUNCH_PID" -f "$LAUNCH_LOG" &
    TAIL_PID=$!

    while true; do
        [ "$STOP_REQUESTED" = "1" ] && break

        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            wait "$LAUNCH_PID"
            EXIT_CODE=$?
            log "Launch exited code=$EXIT_CODE"
            break
        fi

        if tail -n 50 "$LAUNCH_LOG" | \
            grep -qiE "(failed to open|orbbec.*error|device failed|\[error\])"
        then
            log "Orbbec error detected"
            break
        fi

        sleep 0.2
    done

    cleanup_launch
    rm -f "$LAUNCH_LOG"

    [ "$STOP_REQUESTED" = "1" ] && break

    log "Restart after 2 seconds..."
    sleep 2
done

cleanup_launch
rm -f "$PIDFILE"
log "Watchdog exited"
