import time
import sys
import json
import zmq
import zmq.eventloop.zmqstream

sys.path.append('/home/pi/sphero-sdk-raspberrypi-python')

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices
import qwiic
import pi_servo_hat

rvr = SpheroRvrObserver()
ctx = zmq.Context()
hat = pi_servo_hat.PiServoHat()
hat.restart()

sock_cmd = ctx.socket(zmq.SUB)
stream_cmd = zmq.eventloop.zmqstream.ZMQStream(sock_cmd)

linear_speed = 2.0                  # Auto drive speed
driving_manual = False              # Variables to handle manual mode timeout
last_cmd_time = time.time()         
cmd_timeout_treshold = 1          
distance_treshold = 50              # Treshold [mm]


# Callback for commands
# Try parsing JSON. If it fails, ignore the message
def on_cmd(msg):
    global driving_manual
    global last_cmd_time
    global linear_speed
    #print("Message received: ", msg[0])
    #return
    try:
        j = json.loads(msg[0])
        cmd = j['command']
        # print()
        # print(cmd)
        # print(msg)
        params = j['parameters']

        if cmd == "manual":
            # If just entering manual mode, stop to be sure RVR is not driving automatically
            if not driving_manual:
                rvr.drive_stop()
            rvr.drive_rc_si_units(float(params["speed_rotation"]), float(params["speed_forward"]), 1, 1)
            driving_manual = True

        elif cmd == "auto":
            driving_manual = False
            rvr.drive_to_position_si(float(params["yaw"]), float(params["x"]), float(params["y"]), linear_speed)

        elif cmd == "servo":
            hat.move_servo_position(int(params["channel"]), float(params["angle_deg"]), swing=180)
            
        elif cmd == "set_colour":
            rvr.led_control.set_all_leds_rgb(int(params["r"]), int(params["g"]), int(params["b"]))

        last_cmd_time = time.time()

    except Exception as e:
        print("\nNo valid command found")
        print(e)
        print(msg)
        return

def check_connection():
    global driving_manual
    global last_cmd_time
    global cmd_timeout_treshold
    if driving_manual and (time.time() - last_cmd_time) > cmd_timeout_treshold:
        print("Timeout")
        rvr.drive_rc_si_units(0, 0, 1, 1)
        driving_manual = False
        rvr.led_control.set_all_leds_rgb(0,0,0)
    # Register next call
    stream_cmd.io_loop.call_later(0.8*cmd_timeout_treshold, check_connection)
    

def main():
    global driving_manual
    global last_cmd_time
    global cmd_timeout_treshold

    print("Starting RVR server")
    rvr.wake()
    time.sleep(2)
    print("RVR is awake")
    rvr.reset_locator_x_and_y()
    rvr.reset_yaw()
    rvr.led_control.set_all_leds_rgb(0,0,0)

    sock_cmd.bind('tcp://*:5555')
    sock_cmd.subscribe("")
    sock_cmd.setsockopt(zmq.CONFLATE, 1) # Only keep the last message in the queue
    print("Socket bound")
    stream_cmd.on_recv(on_cmd)
    stream_cmd.io_loop.call_later(cmd_timeout_treshold, check_connection)
    print("Listening for commands")
    stream_cmd.io_loop.current().start() # Only returns when the loop is stopped
    
# Start main loop if script is run directly
if __name__ == '__main__':
    # Wrap in try-catch to handle 
    try:
        main()
    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')
    finally:
        stream_cmd.io_loop.current().stop()
        sock_cmd.close()
        rvr.sensor_control.clear()
        # Delay to allow RVR issue command before closing
        time.sleep(.5)
        rvr.close()
