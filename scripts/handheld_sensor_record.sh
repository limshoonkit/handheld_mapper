#!/bin/bash
set -e
source /home/$USER/handheld_mapper/install/setup.bash

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_OUTPUT_PATH="/home/$USER/Desktop/data/rosbag_${TIMESTAMP}"

# Use Zenity to select the desired launch file
LAUNCH_FILE=$(zenity --list --title="Select Launch File" \
    --column="Select" \
    "handheld_compressed.launch.py" \
    "handheld_livox_hik.launch.py" \
    "handheld_sensors.launch.py" \
    --width=400 --height=200)

# If user cancels, exit the script
if [[ -z "$LAUNCH_FILE" ]]; then
    echo "No launch file selected. Exiting..."
    exit 1
fi

# Start ROS2 with the selected launch file in background
setsid ros2 launch handheld_bringup $LAUNCH_FILE \
    bag_output_path:=${BAG_OUTPUT_PATH} &
ROS_PID=$!

# Wait for nodes to initialize
sleep 5

# Show a single "Stop Recording" button
zenity --info --text="Recording in progress.\n\nData will be saved to:\n${BAG_OUTPUT_PATH}\n\nClick OK to stop recording." --ok-label="Stop Recording" --width=400

echo "Stopping recording gracefully..."

# Send SIGINT to entire process group
kill -SIGINT -${ROS_PID} 2>/dev/null || true

# Wait for graceful shutdown
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    echo "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

zenity --info --text="Recording stopped successfully!\n\nData saved to:\n${BAG_OUTPUT_PATH}" --width=400
