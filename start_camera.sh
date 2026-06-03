#!/bin/bash
set -u

LOG=~/camera_start.log
TERM_TITLE="Azure Kinect Camera Start"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

if [ "${1:-}" != "--run" ]; then
    if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$TERM_TITLE" -- bash -lc "'$SCRIPT_PATH' --run; exec bash"
        exit 0
    fi

    if [ -n "${DISPLAY:-}" ] && command -v xterm >/dev/null 2>&1; then
        xterm -T "$TERM_TITLE" -e bash -lc "'$SCRIPT_PATH' --run; exec bash" &
        exit 0
    fi

    exec "$SCRIPT_PATH" --run
fi

cleanup_camera_processes() {
    local pattern='ros2 launch camera_bridge k4a_and_serial.launch.py'

    pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
    sleep 2
    pkill -KILL -f "$pattern" >/dev/null 2>&1 || true
}

run_camera_launch() {
    cleanup_camera_processes

    ros2 launch camera_bridge k4a_and_serial.launch.py
    return $?
}

run_log() {
    echo "$1"
}

echo "====================" > "$LOG"
echo "ROS2 Camera Startup" >> "$LOG"
date >> "$LOG"
echo "====================" >> "$LOG"

exec > >(tee -a "$LOG") 2>&1

echo "===================="
echo "ROS2 Camera Startup"
date
echo "===================="

sleep 10

set +u
source /opt/ros/humble/setup.bash
source ~/workspace/camera_ws/install/setup.bash
set -u

while [ ! -e /dev/azurekinect ]; do
    run_log "Waiting serial..."
    sleep 1
done

run_log "Serial OK"

while true
do
    if run_camera_launch; then
        run_log "ROS launch exited, restarting..."
    else
        run_log "ROS launch failed, cleaning up and restarting..."
    fi

    sleep 5
done
