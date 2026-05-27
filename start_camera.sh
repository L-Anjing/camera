#!/bin/bash

LOG=~/camera_start.log

echo "====================" > $LOG
echo "ROS2 Camera Startup" >> $LOG
date >> $LOG
echo "====================" >> $LOG

sleep 10

source /opt/ros/humble/setup.bash
source ~/workspace/camera_ws/install/setup.bash

while [ ! -e /dev/azurekinect ]; do
    echo "Waiting serial..." >> $LOG
    sleep 1
done

echo "Serial OK" >> $LOG

while true
do
    ros2 launch camera_bridge k4a_and_serial.launch.py \
    >> $LOG 2>&1

    echo "ROS crashed, restarting..." >> $LOG
    sleep 5
done