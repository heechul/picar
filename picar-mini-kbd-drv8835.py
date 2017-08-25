#!/usr/bin/python
import os
import time
import atexit
import serial
import cv2
import math
import numpy as np
import pygame
import sys
import rospy

from pololu_drv8835_rpi import motors, MAX_SPEED

def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

motors.setSpeeds(0, 0)
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 15.0, (640,480))
keyfile = open('out-key.csv', 'w+')
keyfile_btn = open('out-key-btn.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")
keyfile_btn.write("ts_micro,frame,btn,speed\n")
rec_start_time = 0
SET_SPEED = MAX_SPEED * 5 / 10 
cur_speed = SET_SPEED
print "MAX speed:", MAX_SPEED
print "cur speed:", cur_speed

def stop():
    global cur_speed
    cur_speed = 0
    motors.motor2.setSpeed(int(cur_speed))
        
def ffw():
    global cur_speed
    # cur_speed = min(MAX_SPEED, cur_speed + MAX_SPEED/10)
    cur_speed = SET_SPEED    
    motors.motor2.setSpeed(int(cur_speed))

def rew():
    global cur_speed        
    # cur_speed = max(-MAX_SPEED, cur_speed - MAX_SPEED/10)
    cur_speed = SET_SPEED
    motors.motor2.setSpeed(-int(cur_speed))

def center():
    motors.motor1.setSpeed(0)
def left():
    motors.motor1.setSpeed(MAX_SPEED)
def right():
    motors.motor1.setSpeed(-MAX_SPEED)
        
def turnOff():
    stop()
    center()

atexit.register(turnOff)

view_video = False
frame_id = 0
null_frame = np.zeros((160,120,3), np.uint8)
cv2.imshow('frame', null_frame)

angle = 0.0
btn   = 107
period = 50 # ms

if len(sys.argv) == 2:
    SET_SPEED = int(sys.argv[1])
    print "Set new speed: ", SET_SPEED

rospy.init_node('my_node_name')
r = rospy.Rate(20) # 10hz

prev_ts = int(time.time() * 1000)

while not rospy.is_shutdown():
    ts = int(time.time() * 1000)
    
    # read a frame
    ret, frame = cap.read()

    if view_video == True:
        cv2.imshow('frame', frame)

    ch = cv2.waitKey(1) & 0xFF

    if ch == ord('j'):
        left()
        print "left"
        angle = deg2rad(-30)
        btn   = ch
    elif ch == ord('k'):
        center()
        print "center"
        angle = deg2rad(0)
        btn   = ch                
    elif ch == ord('l'):
        right()
        print "right"
        angle = deg2rad(30)
        btn   = ch               
    elif ch == ord('a'):
        ffw()
        print "accel"
    elif ch == ord('s'):
        stop()
        print "stop"                
        btn   = ch
    elif ch == ord('z'):
        rew()
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
        frame_id = frame_id + 1
        
        # write input (angle)
        str = "{},{},{}\n".format(ts, frame_id, angle)
        keyfile.write(str)
        
        # write input (button: left, center, stop, speed)
        str = "{},{},{},{}\n".format(ts, frame_id, btn, cur_speed)
        keyfile_btn.write(str)
        
        # write video stream
        vidfile.write(frame)
        
        if frame_id >= 400:
            print "recorded 400 frames"
            break

    print ts, frame_id, angle, btn, (ts - prev_ts)            
    prev_ts = ts
    r.sleep()    

stop()
cap.release()
keyfile.close()
keyfile_btn.close()
vidfile.release()
