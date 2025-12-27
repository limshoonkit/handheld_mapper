#!/bin/bash
set -e
source /home/$USER/handheld_mapper/install/setup.bash

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_OUTPUT_PATH="/home/$USER/Desktop/data/rosbag_${TIMESTAMP}"
SVO_OUTPUT_DIR="/home/$USER/Desktop/data/svo_${TIMESTAMP}"
mkdir -p ${SVO_OUTPUT_DIR}

# Use Zenity to select the desired launch file
LAUNCH_FILE=$(zenity --list --title="Select Launch File" \
    --column="Select" \
    "handheld_compressed_svo.launch.py" \
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

# Start SVO recording if using the SVO launch file
if [[ "$LAUNCH_FILE" == "handheld_compressed_svo.launch.py" ]]; then
    SVO_OUTPUT_PATH="${SVO_OUTPUT_DIR}/zed_recording.svo2"
    echo "Waiting for ZED SVO recording service..."

    # Wait for service to be available (max 30 seconds)
    timeout=60
    while [ $timeout -gt 0 ]; do
        if ros2 service list | grep -q "/zed_node/start_svo_rec"; then
            echo "Service available, starting SVO recording to: ${SVO_OUTPUT_PATH}"
            sleep 1

            # Start SVO recording with compression mode 4 (H.265 Lossless)
            # https://www.stereolabs.com/docs/ros2/record_and_replay_data_with_ros_wrapper
            ros2 service call /zed_node/start_svo_rec zed_msgs/srv/StartSvoRec \
                "{svo_filename: '${SVO_OUTPUT_PATH}', compression_mode: 4}"

            if [ $? -eq 0 ]; then
                echo "SVO recording started successfully!"
                break
            else
                echo "Failed to start SVO recording"
            fi
        fi
        sleep 0.5
        timeout=$((timeout-1))
    done

    if [ $timeout -eq 0 ]; then
        echo "WARNING: ZED service did not become available within 30 seconds"
        zenity --warning --text="Warning: Could not start SVO recording.\nZED service not available.\n\nContinuing with ROS bag only..." --width=400
    fi
fi

# Show a single "Stop Recording" button
if [[ "$LAUNCH_FILE" == "handheld_compressed_svo.launch.py" ]]; then
    zenity --info --text="Recording in progress.\n\nROS Bag: ${BAG_OUTPUT_PATH}\nZED SVO: ${SVO_OUTPUT_PATH}\n\nClick OK to stop recording." --ok-label="Stop Recording" --width=450
else
    zenity --info --text="Recording in progress.\n\nData will be saved to:\n${BAG_OUTPUT_PATH}\n\nClick OK to stop recording." --ok-label="Stop Recording" --width=400
fi

echo "Stopping recording gracefully..."

# Stop SVO recording if using the SVO launch file
if [[ "$LAUNCH_FILE" == "handheld_compressed_svo.launch.py" ]]; then
    echo "Stopping SVO recording..."
    ros2 service call /zed_node/stop_svo_rec std_srvs/srv/Trigger 2>/dev/null || true
    sleep 2
    echo "SVO recording stopped"
fi

# Send SIGINT to entire process group
kill -SIGINT -${ROS_PID} 2>/dev/null || true

# Wait for graceful shutdown
sleep 5

# Check if still running and force kill if necessary
if kill -0 ${ROS_PID} 2>/dev/null; then
    echo "Force stopping remaining processes..."
    kill -SIGKILL -${ROS_PID} 2>/dev/null || true
fi

if [[ "$LAUNCH_FILE" == "handheld_compressed_svo.launch.py" ]]; then
    zenity --info --text="Recording stopped successfully!\n\nROS Bag: ${BAG_OUTPUT_PATH}\nZED SVO: ${SVO_OUTPUT_PATH}" --width=450
else
    zenity --info --text="Recording stopped successfully!\n\nData saved to:\n${BAG_OUTPUT_PATH}" --width=400
fi
