#!/bin/bash
#
# Re-run FAST-LIVO2 on all uosm_campus bags with per-frame output,
# then convert to Global-LVBA format.
#
# Usage: ./run_lvba_data_gen.sh [SEQUENCE_NAME]
#   Without args: process all sequences
#   With arg:     process only the named sequence (e.g. LF-01-D)
#
set -o pipefail

DATASET_ROOT="/home/ubuntu/Desktop/dataset/uosm_campus"
WORKSPACE="/home/ubuntu/Desktop/uosm_cirg/handheld_mapper"
OUTPUT_ROOT="${WORKSPACE}/data/lvba"
FL2_LOG_DIR="${WORKSPACE}/src/FAST-LIVO2/Log"
FL2_CONFIG="${WORKSPACE}/src/handheld_bringup/config/fast-livo2/mid360_lvba.yaml"
CAM_CONFIG="${WORKSPACE}/src/handheld_bringup/config/fast-livo2/camera_pinhole_mid360.yaml"
CONVERTER="${WORKSPACE}/src/handheld_bringup/scripts/fl2_to_lvba.py"
SEQ_NAME="lvba_run"

declare -A BAGS
BAGS[BF-00]="${DATASET_ROOT}/BF-00/rosbag_20251228_085427"
BAGS[LF-01-D]="${DATASET_ROOT}/LF-01-D/rosbag_20251228_062406"
BAGS[LF-01-N]="${DATASET_ROOT}/LF-01-N/rosbag_20251227_135544"
BAGS[LF-02-D]="${DATASET_ROOT}/LF-02-D/rosbag_20251228_061336"
BAGS[LF-02-N]="${DATASET_ROOT}/LF-02-N/rosbag_20260118_133722"
BAGS[LF-03-D]="${DATASET_ROOT}/LF-03-D/rosbag_20251228_134208"
BAGS[UF-01-D]="${DATASET_ROOT}/UF-01-D/rosbag_20251228_055238"
BAGS[UF-01-N]="${DATASET_ROOT}/UF-01-N/rosbag_20251227_131531"
BAGS[UF-02-D]="${DATASET_ROOT}/UF-02-D/rosbag_20251228_060120"
BAGS[UF-02-N]="${DATASET_ROOT}/UF-02-N/rosbag_20251227_132632"

if [ "${1:-}" != "" ]; then
    SEQUENCES=("$1")
    if [ -z "${BAGS[$1]+x}" ]; then
        echo "ERROR: Unknown sequence '$1'"
        echo "Available: ${!BAGS[*]}"
        exit 1
    fi
else
    SEQUENCES=(BF-00 LF-01-D LF-01-N LF-02-D LF-02-N LF-03-D UF-01-D UF-01-N UF-02-D UF-02-N)
fi

echo "============================================"
echo "  FAST-LIVO2 -> Global-LVBA data generation"
echo "============================================"
echo "Dataset root: ${DATASET_ROOT}"
echo "Output root:  ${OUTPUT_ROOT}"
echo "Sequences:    ${SEQUENCES[*]}"
echo ""

echo "Removing old output: ${DATASET_ROOT}/0_output"
rm -rf "${DATASET_ROOT}/0_output"

source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"

mkdir -p "${OUTPUT_ROOT}"

cleanup_fl2_node() {
    if [ -n "${FL2_PID:-}" ] && kill -0 "$FL2_PID" 2>/dev/null; then
        echo "  Sending SIGINT to FAST-LIVO2 (pid ${FL2_PID})..."
        kill -INT "$FL2_PID" 2>/dev/null || true
        sleep 2
        if kill -0 "$FL2_PID" 2>/dev/null; then
            kill -9 "$FL2_PID" 2>/dev/null || true
        fi
    fi
}

process_sequence() {
    local SEQ="$1"
    local BAG="${BAGS[$SEQ]}"
    local OUT="${OUTPUT_ROOT}/${SEQ}"

    echo ""
    echo "=============================="
    echo "  Processing: ${SEQ}"
    echo "  Bag: ${BAG}"
    echo "=============================="

    if [ ! -d "${BAG}" ]; then
        echo "WARNING: Bag not found at ${BAG}, skipping."
        return
    fi

    # Clean FAST-LIVO2 Log/ from previous run
    rm -rf "${FL2_LOG_DIR}/PCD" "${FL2_LOG_DIR}/Colmap" "${FL2_LOG_DIR}/result"
    mkdir -p "${FL2_LOG_DIR}/PCD" "${FL2_LOG_DIR}/result"
    mkdir -p "${FL2_LOG_DIR}/Colmap/images" "${FL2_LOG_DIR}/Colmap/sparse/0"

    # --- Start FAST-LIVO2 node in background ---
    echo "[${SEQ}] Starting FAST-LIVO2 node..."
    ros2 run fast_livo fastlivo_mapping \
        --ros-args \
        --log-level WARN \
        -p use_sim_time:=true \
        --params-file "${FL2_CONFIG}" \
        -p camera_config:="${CAM_CONFIG}" \
        &
    FL2_PID=$!
    trap cleanup_fl2_node EXIT

    sleep 2

    # --- Play the bag with progress monitor ---
    echo "[${SEQ}] Playing bag..."
    ros2 bag play "${BAG}" --clock -r 1.0 --disable-keyboard-controls &
    BAG_PID=$!

    # Progress monitor: show pose/pcd/image counts while bag plays
    while kill -0 "$BAG_PID" 2>/dev/null; do
        local n_pose n_pcd n_img
        n_pose=$(wc -l < "${FL2_LOG_DIR}/result/${SEQ_NAME}.txt" 2>/dev/null || echo 0)
        n_pcd=$(find "${FL2_LOG_DIR}/PCD" -maxdepth 1 -name "*.pcd" 2>/dev/null | wc -l || echo 0)
        n_img=$(find "${FL2_LOG_DIR}/Colmap/images" -maxdepth 1 -name "*.png" 2>/dev/null | wc -l || echo 0)
        printf "\r  [${SEQ}] poses: %-6s  pcds: %-6s  images: %-6s" "$n_pose" "$n_pcd" "$n_img"
        sleep 5
    done
    wait "$BAG_PID" 2>/dev/null || true
    echo ""
    echo "[${SEQ}] Bag playback finished."

    # Give FAST-LIVO2 time to flush remaining data
    echo "[${SEQ}] Waiting for FAST-LIVO2 to finish processing..."
    sleep 5

    # Gracefully stop FAST-LIVO2: SIGINT -> wait -> SIGTERM -> wait -> SIGKILL
    echo "[${SEQ}] Stopping FAST-LIVO2..."
    kill -INT "$FL2_PID" 2>/dev/null || true
    for i in $(seq 1 15); do
        if ! kill -0 "$FL2_PID" 2>/dev/null; then break; fi
        sleep 1
    done
    if kill -0 "$FL2_PID" 2>/dev/null; then
        echo "[${SEQ}] SIGINT didn't stop it, sending SIGTERM..."
        kill -TERM "$FL2_PID" 2>/dev/null || true
        sleep 3
    fi
    if kill -0 "$FL2_PID" 2>/dev/null; then
        echo "[${SEQ}] Forcing kill..."
        kill -9 "$FL2_PID" 2>/dev/null || true
        sleep 1
    fi
    wait "$FL2_PID" 2>/dev/null || true
    FL2_PID=""
    trap - EXIT

    # --- Verify output exists ---
    if [ ! -f "${FL2_LOG_DIR}/result/${SEQ_NAME}.txt" ]; then
        echo "ERROR: [${SEQ}] No EVO trajectory produced. Check FAST-LIVO2 logs."
        echo "  Expected: ${FL2_LOG_DIR}/result/${SEQ_NAME}.txt"
        ls -la "${FL2_LOG_DIR}/result/" 2>/dev/null || true
        return
    fi

    local N_PCD
    N_PCD=$(find "${FL2_LOG_DIR}/PCD" -name "*.pcd" 2>/dev/null | wc -l || echo 0)
    local N_IMG
    N_IMG=$(find "${FL2_LOG_DIR}/Colmap/images" -name "*.png" 2>/dev/null | wc -l || echo 0)
    local N_POSE
    N_POSE=$(wc -l < "${FL2_LOG_DIR}/result/${SEQ_NAME}.txt")
    echo "[${SEQ}] Output: ${N_POSE} poses, ${N_PCD} PCDs, ${N_IMG} images"

    # --- Convert to LVBA format ---
    echo "[${SEQ}] Converting to LVBA format..."
    if ! python3 "${CONVERTER}" "${FL2_LOG_DIR}" "${OUT}" --seq-name "${SEQ_NAME}"; then
        echo "WARNING: [${SEQ}] Converter failed, skipping remaining steps."
        return
    fi

    if [ -f "${FL2_LOG_DIR}/result/${SEQ_NAME}.txt" ]; then
        cp "${FL2_LOG_DIR}/result/${SEQ_NAME}.txt" "${OUT}/pose_fl2.txt"
    fi

    echo "[${SEQ}] Done -> ${OUT}"
}

for SEQ in "${SEQUENCES[@]}"; do
    process_sequence "${SEQ}"
done

echo ""
echo "============================================"
echo "  All done! LVBA datasets at: ${OUTPUT_ROOT}"
echo "============================================"
echo ""
echo "Output structure:"
echo "  ${OUTPUT_ROOT}/<SEQUENCE>/all_pcd_body/  (per-frame body-frame PCDs + lidar_poses.txt)"
echo "  ${OUTPUT_ROOT}/<SEQUENCE>/all_image/     (per-frame images + image_poses.txt)"
echo ""
echo "To run Global-LVBA refinement on a sequence:"
echo "  1. Edit data_config.data_path in config/global-lvba/config.yaml"
echo "     to point to: ${OUTPUT_ROOT}/<SEQUENCE>/"
echo "  2. ros2 launch handheld_bringup global_lvba_offline.launch.py"
