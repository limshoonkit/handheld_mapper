#!/bin/bash
set -e
source /home/$USER/handheld_mapper/install/setup.bash

STORAGE_FORMAT="mcap"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_OUTPUT_PATH="/home/$USER/Desktop/data/rosbag_${TIMESTAMP}"
SVO_OUTPUT_DIR="/home/$USER/Desktop/data/svo_${TIMESTAMP}"
SVO_OUTPUT_PATH="${SVO_OUTPUT_DIR}/zed_recording.svo2"
mkdir -p ${SVO_OUTPUT_DIR}

# Start ROS2 launch file in background (without rosbag recording)
setsid ros2 launch handheld_bringup handheld_compressed_svo.launch.py &
ROS_PID=$!

# Wait for nodes to initialize
sleep 5

echo "Waiting for ZED SVO recording service..."

# Wait for service to be available (max 60 seconds)
timeout=120
while [ $timeout -gt 0 ]; do
    if ros2 service list | grep -q "/zed_node/start_svo_rec"; then
        echo "Service available!"
        sleep 1
        break
    fi
    sleep 0.5
    timeout=$((timeout-1))
done

if [ $timeout -eq 0 ]; then
    echo "ERROR: ZED service did not become available"
    zenity --error --text="Error: ZED service not available.\n\nCannot start recording." --width=400
    kill -SIGINT -${ROS_PID} 2>/dev/null || true
    exit 1
fi

# Start rosbag recording in background with selected storage format
echo "Starting rosbag recording to: ${BAG_OUTPUT_PATH} (format: ${STORAGE_FORMAT})"


MCAP_CONFIG="/home/$USER/handheld_mapper/src/handheld_bringup/config/mcap_writer_options_compression.yaml"
ros2 bag record -o ${BAG_OUTPUT_PATH} \
    --storage ${STORAGE_FORMAT} \
    --storage-config-file ${MCAP_CONFIG} \
    /livox/imu \
    /livox/lidar \
    /left_camera/image \
    /zed_node/imu/data \
    /zed_node/left/color/rect/image \
    /zed_node/right/color/rect/image &
ROSBAG_PID=$!

# Small delay to ensure rosbag is ready
sleep 0.5

# Start SVO recording (almost simultaneous with rosbag)
echo "Starting SVO recording to: ${SVO_OUTPUT_PATH}"
ros2 service call /zed_node/start_svo_rec zed_msgs/srv/StartSvoRec \
    "{svo_filename: '${SVO_OUTPUT_PATH}', compression_mode: 4, bitrate: 60000}"

if [ $? -eq 0 ]; then
    echo "SVO recording started successfully!"
else
    echo "Failed to start SVO recording"
    zenity --error --text="Error: Failed to start SVO recording.\n\nStopping..." --width=400
    kill -SIGINT ${ROSBAG_PID} 2>/dev/null || true
    kill -SIGINT -${ROS_PID} 2>/dev/null || true
    exit 1
fi

# Show recording dialog
zenity --info --text="Recording in progress.\n\nROS Bag: ${BAG_OUTPUT_PATH}\nStorage Format: ${STORAGE_FORMAT}\nZED SVO: ${SVO_OUTPUT_PATH}\n\nClick OK to stop recording." --ok-label="Stop Recording" --width=450

echo "Stopping recording gracefully..."

# Stop SVO recording
echo "Stopping SVO recording..."
ros2 service call /zed_node/stop_svo_rec std_srvs/srv/Trigger 2>/dev/null || true
sleep 1
echo "SVO recording stopped"

# Stop rosbag recording
echo "Stopping rosbag recording..."
kill -SIGINT ${ROSBAG_PID} 2>/dev/null || true
sleep 2
echo "Rosbag recording stopped"

# Send SIGINT to entire ROS2 launch process group
kill -SIGINT -${ROS_PID} 2>/dev/null || true

# Wait for graceful shutdown
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    echo "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

zenity --info --text="Recording stopped successfully!\n\nROS Bag: ${BAG_OUTPUT_PATH}\nZED SVO: ${SVO_OUTPUT_PATH}" --width=450
