import sys
import time

sys.path.append('/home/pi/sphero-sdk-raspberrypi-python') # For raspi

# Requires packages - aiohttp, pyserial-asyncio, idna, (spidev - for GPS?)
from sphero_sdk import SpheroRvrObserver
import qwiic
import pi_servo_hat

rvr = SpheroRvrObserver()
mux = qwiic.QwiicTCA9548A()
hat = pi_servo_hat.PiServoHat()

rvr.wake()
time.sleep(2)

rvr.get_encoder_counts(lambda c : print(c))
time.sleep(1)
rvr.get_drive_target_slew_parameters(lambda p : print(p))
time.sleep(1)

# Main code
# Blink RGB
for i in range(3):
    rvr.led_control.set_all_leds_rgb(255, 0, 0)
    time.sleep(0.1)
    rvr.led_control.set_all_leds_rgb(0, 255, 0)
    time.sleep(0.1)
    rvr.led_control.set_all_leds_rgb(0, 0, 255)
    time.sleep(0.1)
    rvr.led_control.set_all_leds_rgb(0, 0, 0)

# Set position and angle to (0,0), 0
rvr.reset_locator_x_and_y()
rvr.reset_yaw()
time.sleep(0.1)

# Flag used to wait for move completion
move_completed = False
# Handler for completion of XY position drive moves
def on_xy_position_drive_result_notify_handler(response):
    global move_completed
    move_completed = True
    print('Move completed, response:', response)
rvr.on_xy_position_drive_result_notify(handler=on_xy_position_drive_result_notify_handler)

# XY positioning wrapper for blocking
def drive_to_position_wait_to_complete(yaw_angle,x,y,linear_speed,flags):
    global move_completed
    move_completed = False
    print("Driving to ({0},{1}) at {2} degrees".format(x,y,yaw_angle))
    rvr.drive_to_position_si(yaw_angle, x, y, linear_speed, flags)
    
    while not move_completed:
        time.sleep(0.001)

# Drive in a square
p = 0.5
for i in range(1):
    print("P1")
    drive_to_position_wait_to_complete(90, 0, p, 3, 0) # TODO: Requires newest version of SDK, fetch automatically from github?
    print("P2")
    drive_to_position_wait_to_complete(180, -p, p, 3, 0)
    print("P3")
    drive_to_position_wait_to_complete(270, -p, 0, 3, 0)
    print("P4")
    drive_to_position_wait_to_complete(0, 0, 0, 3, 0)

print("Driving longer")
drive_to_position_wait_to_complete(90, 3, 0, 3, 0)
drive_to_position_wait_to_complete(270, -3, 0, 3, 0)
drive_to_position_wait_to_complete(0, 0, 0, 3, 0)

hat.move_servo_position(0, -45+90)
time.sleep(0.5)
hat.move_servo_position(0, 45+90)
time.sleep(0.5)
hat.move_servo_position(0, 0+90)

rvr.close()
