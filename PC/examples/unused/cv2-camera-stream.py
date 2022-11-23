import cv2 as cv
# System update: https://forums.raspberrypi.com/viewtopic.php?t=323279
# Install python3-opencv to get system dependencies
# Uninstall and use pip to get latest versions of numpy and opencv
import zmq
from time import sleep

ctx = zmq.Context()
sock = zmq.Socket(ctx, zmq.PUB)
sock.bind("tcp://*:5555")

cap = cv.VideoCapture(0, cv.CAP_ANY)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1) # As small buffer size as possible to always get latest frame
sleep(1)

print("Stream started, using video backend:", cap.getBackendName())
while True:
    _, frame = cap.read()
    grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, data = cv.imencode('.jpg', grey)
    sock.send(data)

# Try sending one larger in between small frames?



import socket
import cv2 as cv
import numpy as np

# Udp comm
IP = "127.0.0.1"
PORT = 13

#listen to port

#make header to send back

#capture img, convert and send











cap = cv:VideoCapture(0, cv.CAP_ANY)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 205*cap_size)
                      )