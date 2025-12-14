# UoSM Handheld Mapper

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

## Run Fast-Livo2
```
source ./install/setup.bash
ros2 launch handheld_bringup fast_livo2.launch.py

# ros2 bag play ./data/record_20251214_173541/ -p --remap /hik_camera/image:=/left_camera/image
```

collected
header:
  stamp:
    sec: 1765735454
    nanosec: 609688964
  frame_id: ''
height: 1024
width: 1280
encoding: rgb8
is_bigendian: 0
step: 3840
data:
- 53
- 67
- 53
- 53

header:
  stamp:
    sec: 1590203781
    nanosec: 100311350
  frame_id: livox_frame
timebase: 1590203781100311350
point_num: 19968
lidar_id: 192
rsvd:
- 0
- 0
- 0
points:
- offset_time: 0
  x: 3.6640000343322754
  y: -0.3930000066757202
  z: 3.178999900817871
  reflectivity: 47
  tag: 0
  line: 0

from official
header:
  stamp:
    sec: 946685446
    nanosec: 199467063
  frame_id: ''
height: 512
width: 640
encoding: rgb8
is_bigendian: 0
step: 1920
data:
- 146
- 142
- 131
- 145

header:
  stamp:
    sec: 946685556
    nanosec: 699459910
  frame_id: livox_frame
timebase: 946685356699460000
point_num: 24000
lidar_id: 0
rsvd:
- 0
- 0
- 0
points: