import sys
from typing import Tuple
from threading import Thread

import pyzed.sl as sl


SVO_SAVE_PATH = '/home/nvidia/data/svo/zed_recording.svo2'
GRAB_FREQUENCY = 400  # in Hz
SEC_TO_NS = 10**9


def init_camera() -> Tuple[sl.Camera, sl.RuntimeParameters]:
    """Initialize camera and camera params."""
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.coordinate_units = sl.UNIT.METER
    init_params.set_from_svo_file(SVO_SAVE_PATH)
    init_params.svo_real_time_mode = False
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        sys.exit()

    return zed


def grab_images(zed: sl.Camera) -> None:
    runtime_parameters = sl.RuntimeParameters()
    while zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        pass

def record_imu(zed: sl.Camera, record_frequency: int) -> None:
    sensors_data = sl.SensorsData()
    sensor_rate_nanosec = (1 / record_frequency) * SEC_TO_NS
    last_ts_recorded = 0
    while zed.is_opened():
        if not zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            continue
        zed_imu = sensors_data.get_imu_data()
        ts = zed_imu.timestamp.get_nanoseconds()
        if ts > (last_ts_recorded + sensor_rate_nanosec):
            acceleration = zed_imu.get_linear_acceleration()
            angular_velocity = zed_imu.get_angular_velocity()
            er = 1 / ((ts - last_ts_recorded) / SEC_TO_NS)
            row = [round(er), ts, *acceleration, *angular_velocity, sensors_data.image_sync_trigger]
            last_ts_recorded = ts
            print(row)
                

if __name__ == '__main__':

    zed_camera = init_camera()
    sensor_thread_fn = Thread(target=record_imu, args=(zed_camera, GRAB_FREQUENCY))
    sensor_thread_fn.start()
    grab_images(zed_camera)
    zed_camera.close()
    sensor_thread_fn.join()