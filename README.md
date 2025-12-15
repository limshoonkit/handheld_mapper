# UoSM Handheld Mapper

Custom Rig for https://github.com/hku-mars/LIV_handhold_2, support ROS2 Humble. 

FAST-LIVO2 is from [https://github.com/hku-mars/FAST-LIVO2/issues/128]

**Note :-**

1. The livox_ros_driver2 is custom, not from the livox provider. It writes and reads from a timeshare file from home/{user} directory to sync.
2. The FAST-LIVO2 voxelmap is unbounded, hence memory will grow indefinitely [https://github.com/hku-mars/FAST-LIVO2/issues/386] , [https://github.com/hku-mars/FAST-LIVO2/issues/224] , [https://github.com/hku-mars/FAST-LIVO2/issues/258] , [https://github.com/hku-mars/FAST-LIVO2/issues/289]. Jetson Orin with 16GB RAM may force shutdown if map for long period. Other related issue regarding build configurations [https://github.com/hku-mars/FAST-LIVO2/issues/68] , [https://github.com/hku-mars/FAST-LIVO2/issues/101] , [https://github.com/hku-mars/FAST-LIVO2/issues/146]. 



## Hardware / Environment
- Nvidia Jetson Orin NX 16GB
- Jetpack 6.2
- Ubuntu 22.04
- ROS2 Humble
- ZED SDK 4.2.5
- MVS 3.0.1 build20240420 [ARM]

![front](./media/front.jpeg)
![back](./media/back.jpeg)

## Build 

### For SL zed
```
sudo apt update
rosdep update
rosdep install --from-paths src/zed-ros2-wrapper --ignore-src -r -y # install dependencies
```

### Erase all and rebuild
```
./src/livox_ros_driver2/build.sh humble
```

### Build only pecific packages
```
colcon build --packages-select handheld_bringup --symlink-install # eg handheld_bringup
```

## Run Calibration Recording
```
source ./install/setup.bash
ros2 launch handheld_bringup handheld_sensors.launch.py
```
![Display](./media/display.jpeg)

## Run Fast-Livo2
```
source ./install/setup.bash
ros2 launch handheld_bringup fast_livo2.launch.py

# ros2 bag play ./data/record_20251214_173541/ -p --remap /hik_camera/image:=/left_camera/image
```

![Sample0](./media/sample0.jpeg)
![Sample1](./media/sample1.jpeg)
![Sample2](./media/sample2.jpeg)
