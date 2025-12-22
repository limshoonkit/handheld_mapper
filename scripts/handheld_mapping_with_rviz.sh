#!/bin/bash
set -e

# Create log file with timestamp
LOG_FILE="/tmp/handheld_mapping_rviz_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG_FILE"

# Log function
log() {
    echo "[$(date +%H:%M:%S)] $1" | tee -a "$LOG_FILE"
}

log "=== Starting Handheld Mapping with RViz ==="
log "Sourcing ROS2 setup..."
source /home/$USER/handheld_mapper/install/setup.bash

# RViz enabled, no recording
RECORD_BAG="False"
USE_RVIZ="True"

# Check RViz config exists
RVIZ_CONFIG="/home/nvidia/handheld_mapper/install/fast_livo/share/fast_livo/rviz_cfg/fast_livo2.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    log "WARNING: RViz config file not found at $RVIZ_CONFIG"
    zenity --warning --text="RViz config file not found!\nPath: $RVIZ_CONFIG" --width=400
else
    log "RViz config found: $RVIZ_CONFIG"
fi

# Start ROS2 with RViz enabled
log "Launching ROS2 with RViz enabled (no recording)"
setsid ros2 launch handheld_bringup fast_livo2.launch.py record_bag:=$RECORD_BAG use_rviz:=$USE_RVIZ >> "$LOG_FILE" 2>&1 &
ROS_PID=$!
log "ROS2 launched with PID: $ROS_PID"

# Wait for services to become available
log "Waiting for services to start..."
sleep 3

# Check if process is still running
if kill -0 ${ROS_PID} 2>/dev/null; then
    log "ROS2 process is running (PID: $ROS_PID)"

    # Check if RViz is running
    sleep 2
    if pgrep -x "rviz2" > /dev/null; then
        log "RViz2 is running successfully"
    else
        log "WARNING: RViz2 is not running! Check log file for errors."
        zenity --warning --text="RViz2 failed to start!\n\nCheck log file:\n$LOG_FILE" --width=400 --timeout=5
    fi
else
    log "ERROR: ROS2 process died immediately!"
    zenity --error --text="ROS2 failed to start!\n\nCheck log file:\n$LOG_FILE" --width=400
    exit 1
fi

# Show a single "Stop Mapping" button
zenity --info --text="Mapping with RViz in progress.\n\nClick OK to stop mapping.\n\nLog file: $LOG_FILE" --ok-label="Stop Mapping" --width=400

# Send SIGINT to entire process group
log "Stopping mapping (sending SIGINT to process group)..."
kill -SIGINT -${ROS_PID} 2>/dev/null || true

# Wait for graceful shutdown
log "Waiting for graceful shutdown..."
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    log "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

log "Mapping stopped successfully!"
zenity --info --text="Mapping stopped successfully!\n\nLog saved to:\n$LOG_FILE" --width=400
