import zmq
from io import BytesIO
from time import sleep
from picamera import PiCamera

ctx = zmq.Context()
sock = zmq.Socket(ctx, zmq.RADIO)
sock.bind("udp://*:5555")

# Create the in-memory stream
stream = BytesIO()
camera = PiCamera()
camera.resolution=(320,240)
camera.framerate=24
camera.start_preview()
sleep(2) # Wait for camera to initialize - from example code


for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
    # Rewind the stream and send the image data over the wire
    stream.seek(0)
    sock.send(stream.read())
    # Reset the stream for the next capture
    stream.seek(0)
    stream.truncate()
    #print("Frame sent")
