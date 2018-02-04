#!/usr/bin/python
import os
import time
import atexit
import serial
import cv2
import math
import numpy as np
import sys
from threading import Thread
import params
actuator = __import__(params.actuator)

def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

def g_tick():
    t = time.time()
    count = 0
    while True:
        count += 1
        yield max(t + count*period - time.time(),0)

class Camera:
    def __init__(self, res=(320, 240), fps=30):
        print "Initilize camera."
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, res[0]) # width
        self.cap.set(4, res[1]) # height
        self.cap.set(5, fps)
        self.frame = None
        self.enabled = True
        
    def update(self):
        while self.enabled:
            ret, self.frame = self.cap.read() # blocking read.

    def read_frame(self):
        return self.frame

    def shutdown(self):
        print ("Close the camera.")
        self.enabled = False;        
        self.cap.release()

cfg_width = 320
cfg_height = 240
cfg_fps = 30

cam = Camera((cfg_width, cfg_height), cfg_fps)
cam_thr = Thread(target=cam.update, args=())
cam_thr.start()
time.sleep(2)

view_video = False
frame_id = 0

angle = 0.0
btn   = 107
period = 0.05 # sec (=50ms)

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
fourcc = cv2.cv.CV_FOURCC(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 
	cfg_fps, (cfg_width,cfg_height))

keyfile = open('out-key.csv', 'w+')
keyfile_btn = open('out-key-btn.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")
keyfile_btn.write("ts_micro,frame,btn,speed\n")
rec_start_time = 0

default_speed = 50
actuator.init()
actuator.set_speed(default_speed)
atexit.register(actuator.turn_off)

null_frame = np.zeros((cfg_width,cfg_height,3), np.uint8)
cv2.imshow('frame', null_frame)

if len(sys.argv) == 2:
    actuator.set_speed(int(sys.argv[1]))
    print "Set new speed: ", actuator.get_speed()

g = g_tick()
    
while True:
    time.sleep(next(g))    
    ts = time.time()

    frame = cam.read_frame()

    # read a frame
    # ret, frame = cap.read()
        
    if view_video == True:
        cv2.imshow('frame', frame)

    ch = cv2.waitKey(1) & 0xFF

    if ch == ord('j'):
        actuator.left()
        print "left"
        angle = deg2rad(-30)
        btn   = ch
    elif ch == ord('k'):
        actuator.center()
        print "center"
        angle = deg2rad(0)
        btn   = ch                
    elif ch == ord('l'):
        actuator.right()
        print "right"
        angle = deg2rad(30)
        btn   = ch               
    elif ch == ord('a'):
        actuator.ffw()
        print "accel"
    elif ch == ord('s'):
        actuator.stop()
        print "stop"                
        btn   = ch
    elif ch == ord('z'):
        actuator.rew()
        print "reverse"
    elif ch == ord('q'):
        break
    elif ch == ord('r'):
        print "toggle record mode"
        if rec_start_time == 0:
            rec_start_time = ts
        else:
            rec_start_time = 0
    elif ch == ord('t'):
        print "toggle video mode"
        if view_video == False:
            view_video = True
        else:
            view_video = False
	    
                        
    if rec_start_time > 0:
        # increase frame_id
        frame_id += 1
        
        # write input (angle)
        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
        keyfile.write(str)
        
        # write input (button: left, center, stop, speed)
        str = "{},{},{},{}\n".format(int(ts*1000), frame_id, btn, cur_speed)
        keyfile_btn.write(str)
        
        # write video stream
        vidfile.write(frame)
        
        if frame_id >= 1000:
            print "recorded 1000 frames"
            break

    print ("%.3f %d %.3f %d %d(ms)" % (ts, frame_id, angle, btn, int((time.time() - ts)*1000)))
    
actuator.stop()
cam.shutdown()
keyfile.close()
keyfile_btn.close()
vidfile.release()
