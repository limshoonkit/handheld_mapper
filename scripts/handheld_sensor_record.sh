#!/bin/bash
set -e
source /home/nvidia/uosm_handheld_mapper/install/setup.bash
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_OUTPUT_PATH="/home/nvidia/Desktop/data/record_${TIMESTAMP}"

# Start ROS2 in background with its own process group
setsid ros2 launch handheld_bringup handheld_sensors.launch.py \
    bag_output_path:=${BAG_OUTPUT_PATH} &

ROS_PID=$!

# Show a single "Stop Recording" button
zenity --info --text="Recording in progress.\n\nData will be saved to:\n${BAG_OUTPUT_PATH}\n\nClick OK to stop recording." --ok-label="Stop Recording" --width=400

# Send SIGINT to entire process group
echo "Stopping recording gracefully..."
kill -SIGINT -${ROS_PID}

# Wait for graceful shutdown
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    echo "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

zenity --info --text="Recording stopped successfully!\n\nData saved to:\n${BAG_OUTPUT_PATH}" --width=400
