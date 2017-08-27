#!/usr/bin/python
from __future__ import division
import tensorflow as tf
import model
import params
import local_common as cm
import preprocess

import random
import os
import time
import atexit
import serial
import cv2
import math
import numpy as np
import pygame
from pololu_drv8835_rpi import motors, MAX_SPEED
import sys
import rospy

def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

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
    motors.motor2.setSpeed(int(cur_speed))

def center():
    motors.motor1.setSpeed(0)
def left():
    motors.motor1.setSpeed(MAX_SPEED)
def right():
    motors.motor1.setSpeed(-MAX_SPEED)

def get_control(degree):
    if degree < 15 and degree > -15:
        return 0
    elif degree >= 15:
        return 1
    elif degree <= -15:
        return -1

def turnOff():
    stop()
    center()

atexit.register(turnOff)

view_video = False
frame_id = 0
null_frame = np.zeros((160,120,3), np.uint8)
cv2.imshow('frame', null_frame)

## dagger parameter
DEFAULT_BETA = 1.0
beta  = DEFAULT_BETA

## human input variables
angle = 0.0
btn   = ord('k')  # 107 - center
period = 0.05 # sec (=50ms)

total_cnt = 0

if len(sys.argv) == 2:
    SET_SPEED = int(sys.argv[1])
    print "Set new speed: ", SET_SPEED

def g_tick():
    t = time.time()
    count = 0
    while True:
        count += 1
        yield max(t + count*period - time.time(),0)
        
g = g_tick()

while True:
    time.sleep(next(g))    
    ts = time.time()
    
    # 0. read a image frame
    ret, frame = cap.read()

    if view_video == True:
        cv2.imshow('frame', frame)

    total_cnt = total_cnt + 1
    
    # 1. machine input
    img = preprocess.preprocess(frame)
    angle_dnn = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0]
    m_ctrl = get_control(rad2deg(angle_dnn)) # default is to use machine control output

    # 2. human input (safety backup)
    h_ctrl = -99
    ch = cv2.waitKey(1) & 0xFF

    # 2.1. service task
    if ch == ord('a'):
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
    # 2.2. steering key
    elif ch == ord('j'):
        # left 
        angle = deg2rad(-30)
        btn = ch
        h_ctrl = -1
    elif ch == ord('k'):
        # center
        angle = deg2rad(0)
        btn = ch
        h_ctrl = 0
    elif ch == ord('l'):
        # right
        angle = deg2rad(30)
        btn = ch
        h_ctrl = 1

    # 3. decision
    if h_ctrl == m_ctrl or h_ctrl == -99:
        # machine == human or there's no human input 
        ctrl = m_ctrl
    else:
        # if there's human input, use it.
        ctrl = h_ctrl
        print '[M]',
        
    # 4. control
    if ctrl == 0:
        center()
    elif ctrl == 1:
        right()
    elif ctrl == -1:
        left()
        
    # 5. record data
    if ctrl == h_ctrl:
        # human intervened for recovery. must be interesting to record.
        # increase frame_id
        frame_id = frame_id + 1
        
        # write input (angle)
        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
        keyfile.write(str)
        
        # write input (button: left, center, stop, speed)
        str = "{},{},{},{}\n".format(int(ts*1000), frame_id, btn, cur_speed)
        keyfile_btn.write(str)
        
        # write video stream
        vidfile.write(frame)
        
	# print ts, frame_id, angle, btn

        if frame_id >= 200:
            print "recorded 200 frames"
            break

    print ts, frame_id, ctrl, int((time.time() - ts)*1000)
    
cap.release()
keyfile.close()
keyfile_btn.close()
vidfile.release()
