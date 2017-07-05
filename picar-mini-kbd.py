#!/usr/bin/python
import os
import time
import atexit
import serial
import cv2
import math
import numpy as np

def turnOff():
	os.system("killall -HUP mencoder")

atexit.register(turnOff)

cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
ser = serial.Serial('/dev/ttyACM0', 115200)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 20.0, (320,240))
keyfile = open('out-key.csv', 'w+')

rec_start_time = 0

def right():
        ser.write(b'r\n')
        
def left():
        ser.write(b'l\n')
        
def center():
        ser.write(b'c\n')
        
def ffw():
        ser.write(b'f\n')
        
def rew():
        ser.write(b'b\n')
        
def stop():
        ser.write(b's\n')
        
def degree2rad(deg):
        return deg * math.pi / 180.0


frame_id = 0
null_frame = np.zeros((160,120,3), np.uint8)

while (True):
        # read a frame
        ret, frame = cap.read()
        ts = int(time.time() * 1000000)
        
        if ret == False:
                break

        angle = 0.0

        cv2.imshow('frame', frame)
        ch = cv2.waitKey(1) & 0xFF
        
        if ch == ord('j'):
                left()
                angle = degree2rad(-30)
        elif ch == ord('k'):
                center()
        elif ch == ord('l'):
                right()
                angle = degree2rad(30)                
        elif ch == ord('a'):
                ffw()
        elif ch == ord('s'):
                stop()
        elif ch == ord('z'):
                rew()
        elif ch == ord('q'):
                break
	elif ch == ord('r'):
                if rec_start_time == 0:
                        rec_start_time = ts
                else:
                        rec_start_time = 0
        
        print ts, frame_id, angle

        if rec_start_time > 0:
                # increase frame_id
                frame_id = frame_id + 1
                
                # write keyboard input        
                str = "{}, {}, {}\n".format(ts, frame_id, angle)
                keyfile.write(str)
                # write video stream
                vidfile.write(frame)
