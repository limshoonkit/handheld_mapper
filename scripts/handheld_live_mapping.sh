#!/bin/bash
set -e
source /home/nvidia/uosm_handheld_mapper/install/setup.bash


# Start ROS2 with the selected launch file in background
setsid ros2 launch handheld_bringup fast_livo2.launch.py &
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