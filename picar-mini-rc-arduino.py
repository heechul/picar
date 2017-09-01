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
import sys
import string
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
fourcc = cv2.cv.CV_FOURCC(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 20.0, (320,240))

keyfile_rc = open('out-rc.csv', 'w+')
keyfile_rc.write("ts_micro,frame,s_pwm,t_pwm\n")

keyfile = open('out-key.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")

rec_start_time = 0

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

use_dnn = False
view_video = False
frame_id = 0
null_frame = np.zeros((160,120,3), np.uint8)
# cv2.imshow('frame', null_frame)

angle = 0.0
throttle = 0
period = 0.02 # sec (=50ms)
SET_SPEED = 0

left_motor = mh.getMotor(1)
right_motor = mh.getMotor(2)

def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

def rc_to_throttle_angle(str_rc, thr_rc):
    th = (thr_rc - 1355) * 256 / 450
    an = (str_rc - 1355) * 180 / 450 # degree
    return th, an
    
def control_motor_differential(th, an):    
    l_speed = th - an / 5
    r_speed = th + an / 5

    if l_speed > 0:
        left_motor.run(Adafruit_MotorHAT.BACKWARD)
    else:
        left_motor.run(Adafruit_MotorHAT.FORWARD)

    if r_speed > 0:
        right_motor.run(Adafruit_MotorHAT.BACKWARD)
    else:
        right_motor.run(Adafruit_MotorHAT.FORWARD)

    left_motor.setSpeed(int(abs(l_speed)))
    right_motor.setSpeed(int(abs(r_speed)))

    return l_speed, r_speed

if use_dnn:
    sess = tf.InteractiveSession()
    saver = tf.train.Saver()
    model_name = 'model.ckpt'
    model_path = cm.jn(params.save_dir, model_name)
    saver.restore(sess, model_path)

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

ser.flush()
print "start loop"

frame_id = 0

while True:
    time.sleep(next(g))    
    ts = time.time()
    
    ret, frame = cap.read()

    line = ser.readline()
    rc1, rc2 = string.split(line[:-1])
    rc1 = int(rc1)
    rc2 = int(rc2)
    
    if not use_dnn:
        # 1. human input (RC) 
        throttle, angle = rc_to_throttle_angle(rc1, rc2)
    else:
        # 2. machine input
        img = preprocess.preprocess(frame)
        angle_rad = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0]
        angle = rad2deg(angle_rad)
        
    # 3. control_motor(rc1, rc2)
    l_speed, r_speed = control_motor_differential(throttle, angle)

    print ts, frame_id, angle, throttle, rc1, rc2
    
    if rec_start_time > 0:
        # increase frame_id
        frame_id = frame_id + 1
        
        # write input (angle)
        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
        keyfile.write(str)
        
        # write input (rc1, rc2)
        str = "{},{},{},{}\n".format(int(ts*1000), frame_id, rc1, rc2)
        keyfile_rc.write(str)
        
        # write video stream
        vidfile.write(frame)
        
        # if frame_id >= 400:
        #     print "recorded 400 frames"
        #     break
        
cap.release()
keyfile.close()
keyfile_rc.close()
vidfile.release()
