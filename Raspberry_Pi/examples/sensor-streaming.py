import sys
import time

sys.path.append('/home/pi/sphero-sdk-raspberrypi-python')

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices

import qwiic
rvr = SpheroRvrObserver()

# Variables to store VL53L1X sensors
tof_fw = None
tof_bw = None

# Dict to store current RVR position
position = {"x":0, "y":0, "yaw":0}

# Change VL53L1X I2C addresses and initialize ranging
# Sensor on MUX channel 3 should get address 0x28
# Sensor at MUX channel 4 can keep the default 0x29
def init_tof():
    global tof_fw, tof_bw
    # Get MUX
    mux = qwiic.QwiicTCA9548A()
    mux.disable_all()
    # Enable channel for forward range sensor
    mux.enable_channels([3])
    # Check if already configured
    avail_addresses = qwiic.scan()
    print(avail_addresses)
    if 0x29 in avail_addresses:
        tof_fw = qwiic.QwiicVL53L1X(0x29)
        tof_fw.sensor_init()
        # Change address
        tof_fw.set_i2c_address(0x28)
    else: 
        tof_fw = qwiic.QwiicVL53L1X(0x28)
        tof_fw.sensor_init()

    # Enable channel for backward range sensor
    mux.enable_channels([3, 4])
    tof_bw = qwiic.QwiicVL53L1X(0x29)
    tof_bw.sensor_init()

    # Enable sensors
    time.sleep(0.1)
    tof_fw.start_ranging()
    tof_bw.start_ranging()

# Callback functions for sensor data streaming service
def locator_handler(data):
    if data['Locator']['is_valid']:
        global position
        position['x'] = data['Locator']['X']
        position['y'] = data['Locator']['Y']

def imu_handler(data):
    if data['IMU']['is_valid']:
        global position
        position['yaw'] = data['IMU']['Yaw']


def main():
    rvr.wake()
    init_tof()
    # Give RVR time to wake up
    time.sleep(2)
    # Reset position
    rvr.drive_control.reset_heading()
    rvr.reset_locator_x_and_y()
    time.sleep(0.5)

    # Bind streaming services to handler functions
    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler
    )
    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler
    )
    rvr.sensor_control.start(interval=100) # Start streaming at fixed interval (ms)

    while True:
        # Delay to allow RVR to stream sensor data
        time.sleep(0.2)
        fw_range = tof_fw.get_distance()
        bw_range = tof_bw.get_distance() 

        print(  "X: " + f'{position["x"]:.3f}' + "m" +
                "\tY: " + f'{position["y"]:.3f}' + "m" +
                "\tYaw: " + f'{position["yaw"]:.1f}' + "deg" + 
                "\tRange F: " + f'{fw_range*1e-3:.3f}' + "mm" +
                "\tRange B: " + f'{bw_range*1e-3:.3f}' + "mm"
            )


# Start main loop if script is run directly
if __name__ == '__main__':
    # Wrap in try-catch to handle 
    try:
        main()
    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')
    finally:
        rvr.sensor_control.clear()
        tof_fw.stop_ranging()
        tof_bw.stop_ranging()
        # Delay to allow RVR issue command before closing
        time.sleep(.5)
        rvr.close()
