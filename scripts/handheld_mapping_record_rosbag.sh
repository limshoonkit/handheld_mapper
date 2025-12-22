#!/bin/bash
set -e

# Create log file with timestamp
LOG_FILE="/tmp/handheld_mapping_rosbag_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG_FILE"

# Log function
log() {
    echo "[$(date +%H:%M:%S)] $1" | tee -a "$LOG_FILE"
}

log "=== Starting Handheld Mapping with Rosbag Recording ==="
log "Sourcing ROS2 setup..."
source /home/$USER/handheld_mapper/install/setup.bash

# Recording enabled, RViz disabled
RECORD_BAG="True"
USE_RVIZ="False"

# Start ROS2 with rosbag recording
log "Launching ROS2 with rosbag recording (RViz disabled)"
setsid ros2 launch handheld_bringup fast_livo2.launch.py record_bag:=$RECORD_BAG use_rviz:=$USE_RVIZ >> "$LOG_FILE" 2>&1 &
ROS_PID=$!
log "ROS2 launched with PID: $ROS_PID"

# Wait for services to become available
log "Waiting for services to start..."
sleep 3

# Check if process is still running
if kill -0 ${ROS_PID} 2>/dev/null; then
    log "ROS2 process is running (PID: $ROS_PID)"

    # Check if rosbag recording started
    sleep 2
    if pgrep -f "ros2 bag record" > /dev/null; then
        log "Rosbag recording is running"
    else
        log "WARNING: Rosbag recording may not have started! Check log file."
        zenity --warning --text="Rosbag recording may not have started!\n\nCheck log file:\n$LOG_FILE" --width=400 --timeout=5
    fi
else
    log "ERROR: ROS2 process died immediately!"
    zenity --error --text="ROS2 failed to start!\n\nCheck log file:\n$LOG_FILE" --width=400
    exit 1
fi

# Show a single "Stop Mapping" button
zenity --info --text="Mapping with rosbag recording in progress.\n\nRViz is disabled for performance.\n\nClick OK to stop mapping.\n\nLog file: $LOG_FILE" --ok-label="Stop Recording" --width=400

# Send SIGINT to entire process group
log "Stopping mapping and rosbag recording (sending SIGINT to process group)..."
kill -SIGINT -${ROS_PID} 2>/dev/null || true

# Wait for graceful shutdown
log "Waiting for graceful shutdown and rosbag finalization..."
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    log "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

log "Mapping and recording stopped successfully!"
zenity --info --text="Mapping and recording stopped successfully!\n\nRosbag saved to:\n/home/nvidia/Desktop/data/fast_livo2_bag/\n\nLog saved to:\n$LOG_FILE" --width=500
