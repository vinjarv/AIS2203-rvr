import json
import sys
import time
import zmq
import pi_servo_hat as psh

sys.path.append('/home/pi/sphero-sdk-raspberrypi-python')

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices

import qwiic
rvr = SpheroRvrObserver()


#Store VL53L1X sensors
tof_fw = None
tof_bw = None

#Servo addresses
SERVO_1_ADR = 0
SERVO_2_ADR = 1
SERVO_3_ADR = 2
SERVO_4_ADR = 3



position = {"x":0, "y":0, "yaw":0}



context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://0.0.0.0:5555")



def init_tof():

    global tof_fw, tof_bw
    mux = qwiic.QwiicTCA9548A()
    mux.disable_all()
    mux.enable_channels([3])
    # Check if it is already configured
    avail_addresses = qwiic.scan()
    if 0x29 in avail_addresses:
        tof_fw = qwiic.QwiicVL53L1X(0x29)
        tof_fw.sensor_init()
        tof_fw.set_i2c_address(0x28)
    else:
        tof_fw = qwiic.QwiicVL53L1X(0x28)
        tof_fw.sensor_init()

    # Enable channel for backward range sensor
    mux.enable_channels([3,4])
    tof_bw = qwiic.QwiicVL53L1X(0x29)
    tof_bw.sensor_init()

    # Enable sensors
    time.sleep(0.1)
    tof_fw.start_ranging()
    tof_bw.start_ranging()

def init_servos():
    servos = []
    for i in range(4):
        servos.append(psh.PiServoHat())
    return servos

# Callback functions for sensor data
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

    servos = init_servos()


    rvr.wake()
    init_tof()
    time.sleep(2)
    #reset position
    rvr.drive_control.reset_heading()
    rvr.reset_locator_x_and_y()
    time.sleep(0.5)

    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler
    )
    rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler
    )
    rvr.sensor_control.start(interval=100)


    while True:
        time.sleep(0.2)
        fw_range = tof_fw.get_distance()
        bw_range = tof_bw.get_distance()

        # Read servo positions
        servo_pos = []
        for i in range(len(servos)):
            servo_pos.append(servos[i].get_servo_position(SERVO_1_ADR))



        # Creates JSON object

        sensor_data  = {
           "X": f'{position["x"]:.3f}',
           "Y": f'{position["y"]:.3f}',
           "Yaw": f'{position["yaw"]:.1f}',
           "Range F": f'{fw_range*1e-3:.3f}',
           "Range B": f'{bw_range*1e-3:.3f}',
           "Servo 1A": f'{servo_pos[0]}',
           "Servo 1B": f'{servo_pos[1]}',
           "Servo 2A": f'{servo_pos[2]}',
           "Servo 2B": f'{servo_pos[3]}'
}

        json.str = json.dumps(sensor_data)
        socket.send_string(json.str)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')
    finally:
        rvr.sensor_control.clear()
        tof_fw.stop_ranging()
        tof_bw.stop_ranging()
        time.sleep(.5)
        rvr.close

