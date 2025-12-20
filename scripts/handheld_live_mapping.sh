#!/bin/bash
set -e
source /home/$USER/handheld_mapper/install/setup.bash

# Ask user if they want to record rosbag
zenity --question --text="Do you want to record a rosbag?\n\nNote: RViz will be disabled when recording." --width=400

if [ $? -eq 0 ]; then
    # User clicked "Yes" - record rosbag and disable rviz
    RECORD_BAG="True"
    USE_RVIZ="False"
    echo "Recording rosbag (RViz disabled)"
else
    # User clicked "No" - don't record rosbag, enable rviz
    RECORD_BAG="False"
    USE_RVIZ="True"
    echo "Not recording rosbag (RViz enabled)"
fi

# Start ROS2 with the selected launch file in background
setsid ros2 launch handheld_bringup fast_livo2.launch.py record_bag:=$RECORD_BAG use_rviz:=$USE_RVIZ &
ROS_PID=$!

# Wait for services to become available
sleep 3

# Show a single "Stop Mapping" button
zenity --info --text="Mapping in progress.\n\nClick OK to stop mapping." --ok-label="Stop Mapping" --width=400

# Send SIGINT to entire process group
kill -SIGINT -${ROS_PID} 2>/dev/null || true

# Wait for graceful shutdown
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    echo "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

zenity --info --text="Mapping stopped successfully!\n" --width=400