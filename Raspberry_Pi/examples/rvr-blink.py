import sys
import time

sys.path.append('/home/pi/sphero-sdk-raspberrypi-python') # For raspi

# Requires packages - aiohttp, pyserial-asyncio
from sphero_sdk import SpheroRvrObserver
import qwiic            # Not used, just to test that python env is configured correctly
import pi_servo_hat     # ^

rvr = SpheroRvrObserver()

mux = qwiic.QwiicTCA9548A()
hat = pi_servo_hat.PiServoHat()

rvr.wake()
time.sleep(2)

# Main code
rvr.led_control.set_all_leds_rgb(255, 0, 0)
time.sleep(0.5)
rvr.led_control.set_all_leds_rgb(0, 255, 0)
time.sleep(0.5)
rvr.led_control.set_all_leds_rgb(0, 0, 255)
time.sleep(1)
rvr.led_control.set_all_leds_rgb(0, 0, 0)

rvr.close()
