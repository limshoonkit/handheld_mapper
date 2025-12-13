# Handheld Mapper
Custom Rig for https://github.com/hku-mars/LIV_handhold_2

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

## Run Recording
```
source ./install/setup.bash
ros2 launch handheld_bringup handheld_sensors.launch.py
```

![Display](./media/display.jpeg)