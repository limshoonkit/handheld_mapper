# UoSM Handheld Mapper

Custom Rig for https://github.com/hku-mars/LIV_handhold_2, support ROS2 Humble. 

FAST-LIVO2 is based on [https://github.com/yqmy0814/FAST-LIVO2] and [https://github.com/hku-mars/FAST-LIVO2/issues/128]

**Note:-**

1. **livox_ros_driver2** is custom, not the official version from Livox. It reads and writes from a time-share file located in `home/${User}/timeshare` for synchronization. Ensure to set a static IP of `192.168.1.50` for the device connected to the Livox LiDAR. 
2. The **FAST-LIVO2 voxelmap** is unbounded, which means memory usage will grow indefinitely. This could potentially lead to issues such as a forced shutdown when mapping for extended periods on systems with limited memory, like the Jetson Orin (16GB RAM). 
   - Relevant issues: [#386](https://github.com/hku-mars/FAST-LIVO2/issues/386), [#224](https://github.com/hku-mars/FAST-LIVO2/issues/224), [#258](https://github.com/hku-mars/FAST-LIVO2/issues/258), [#289](https://github.com/hku-mars/FAST-LIVO2/issues/289)
   - Build configuration issues: [#68](https://github.com/hku-mars/FAST-LIVO2/issues/68), [#101](https://github.com/hku-mars/FAST-LIVO2/issues/101), [#146](https://github.com/hku-mars/FAST-LIVO2/issues/146)
   
3. **libusb** from MVS (`/opt/MVS/lib/aarch64/libusb-1.0.so.0`) may conflict with the system installation (`/lib/aarch64-linux-gnu/libusb-1.0.so.0`). It is recommended to either delete the MVS version or set the system's `libusb` with the following commands:
```
export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```
4. This repo uses cyclondds and mcap storage. Get it with:
```
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap
```

## Hardware / Environment
- Nvidia Jetson Orin NX 16GB
- Jetpack 6.2
- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ZED SDK 5.1](https://download.stereolabs.com/zedsdk/5.1/l4t36.4/jetsons?_gl=1*fphl2z*_gcl_au*MTY3ODk4NTkwMy4xNzY1OTExMTAz) 
- [MVS 3.0.1 ARM](https://www.hikrobotics.com/en/machinevision/service/download/)
- [Livox-SDK2 v1.2.5](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)
- OpenCV v4.8.0
- PCL  v1.12.1
- Eigen v3.4.0

![front](./media/front.jpeg)
![back](./media/back.jpeg)

## Build 

### Get submodules and dependencies
```
git submodule init
git submodule update
sudo apt update
rosdep update
rosdep install --from-paths src/zed-ros2-wrapper --ignore-src -r -y # install dependencies
rosdep install --from-paths src/livox_ros_driver2 --ignore-src -r -y
rosdep install --from-paths src/mvs_ros_driver2 --ignore-src -r -y
rosdep install --from-paths src/FAST-LIVO2 --ignore-src -r -y
rosdep install --from-paths src/ros2_jetson_stats --ignore-src --rosdistro humble -y

# Install SC-PGO dependencies (GTSAM for pose graph optimization)
sudo apt install ros-humble-gtsam
```

### Erase all and rebuild
```
./src/livox_ros_driver2/build.sh humble
```

### Build only specific packages
```
colcon build --packages-select handheld_bringup --symlink-install # eg handheld_bringup
```

## Run Calibration Recording

1. Get [FastCalib](https://github.com/ichangjian/FAST-Calib-ROS2)
2. Get [kalibr](https://github.com/ethz-asl/kalibr), use the docker
```
source ./install/setup.bash
ros2 launch handheld_bringup handheld_sensors.launch.py
```
![Display](./media/display.jpeg)

## Run Fast-Livo2
**Note :-**

Comment out driver nodes and set preprocess.lidar_type to `3` for bags using standard pointcloud2 msg rather than livox custom msg.
```
source ./install/setup.bash
ros2 launch handheld_bringup fast_livo2.launch.py

# ros2 bag play ./data/record_20251214_173541/ -p --remap /hik_camera/image:=/left_camera/image

# source ./install/setup.bash
# ros2 launch fast_livo mapping_avia.launch.py
# ros2 bag play ./data/fast_livo/test_bag/ros2/ -p
```

![Sample0](./media/sample0.jpeg)
![Sample1](./media/sample1.jpeg)
![Sample2](./media/sample2.jpeg)

## Replay Bag with FAST-LIVO2 + SC-PGO (Loop Closure & Global Optimization)
This launch file runs FAST-LIVO2 for local odometry and SC-PGO for loop closure detection and pose graph optimization. No sensor drivers are included - designed for bag replay.

```NOTE: not working well!!!```

- FAST-LIVO2: LiDAR-Visual-Inertial Odometry (local mapping)
- SC-PGO: Scan Context loop closure + GTSAM pose graph optimization (global consistency)

```bash
source ./install/setup.bash

# Play bag file
ros2 bag play your_bag_file.mcap

# Launch FAST-LIVO2 + SC-PGO
ros2 launch handheld_bringup replay_with_pgo.launch.py
```

**Output files are saved to** `./tmp/sc_pgo/`:
- `optimized_poses_tum.txt` - Optimized poses after loop closure (TUM format: timestamp tx ty tz qx qy qz qw)
- `odom_poses_tum.txt` - Original odometry poses before optimization (TUM format)
- `optimized_poses_kitti.txt` - Optimized poses (KITTI format: 3x4 transformation matrix)
- `odom_poses_kitti.txt` - Original odometry poses (KITTI format)
- `times.txt` - Timestamps for each keyframe
- `Scans/*.pcd` - Individual keyframe point clouds

**TUM format** is compatible with trajectory evaluation tools like `evo`:
```bash
# Evaluate trajectory with evo
evo_ape tum ground_truth.txt ./tmp/sc_pgo/optimized_poses_tum.txt -a --plot --plot_mode xyz
```

**Important Topics:**
- Subscribes: `/sc_pgo_cloud_registered`, `/sc_pgo_odometry` (from FAST-LIVO2)
- Publishes: `/aft_pgo_odom`, `/aft_pgo_path`, `/aft_pgo_map` (optimized results)

## Replay from ZED SVO
Based on https://github.com/stereolabs/ros2_replay_data/blob/main/README.md
```
source ./install/setup.bash
ros2 launch handheld_bringup zed_svo_replay.launch.py \
    svo_file_path:=/home/nvidia/Desktop/data/svo/zed_recording.svo2 \
    bag_output_path:=/home/nvidia/Desktop/data/zed_ros_output
```

Get mcap-cli from https://github.com/foxglove/mcap/releases?q=mcap-cli and use mcap filter to match start and end timing of rosbag
```
mcap filter rosbag_20251227_075258_0.mcap \
  --start 1766821986886082602 \
  --end 1766822024386007309 \
  -o rosbag_trimmed.mcap
```