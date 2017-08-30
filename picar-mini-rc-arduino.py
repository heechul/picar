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
import string
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
fourcc = cv2.cv.CV_FOURCC(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 20.0, (320,240))
keyfile_rc = open('out-rc.csv', 'w+')
keyfile_rc.write("ts_micro,frame,f_pwm,t_pwm\n")
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

view_video = False
frame_id = 0
null_frame = np.zeros((160,120,3), np.uint8)
cv2.imshow('frame', null_frame)

angle = 0.0
period = 0.05 # sec (=50ms)

str_motor = mh.getMotor(1)
thr_motor = mh.getMotor(2)

def control_motor(str_rc, thr_rc):
    if thr_rc > 1350:
        thr_motor.run(Adafruit_MotorHAT.FORWARD)
        thr_motor.setSpeed(int((thr_rc - 1350)*256/450))
    else:
        thr_motor.run(Adafruit_MotorHAT.BACKWARD)
        thr_motor.setSpeed(int(-(thr_rc - 1350)*256/450))

    if str_rc > 1350:
        str_motor.run(Adafruit_MotorHAT.FORWARD)
        str_motor.setSpeed(int((str_rc - 1350)*256/450))
    else:
        str_motor.run(Adafruit_MotorHAT.BACKWARD)
        str_motor.setSpeed(int(-(str_rc - 1350)*256/450))
        
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
 
print "start loop"

while True:
    time.sleep(next(g))    
    ts = time.time()
    ret, frame = cap.read()
    line = ser.readline()
    rc1, rc2 = string.split(line[:-1])
    rc1 = int(rc1)
    rc2 = int(rc2)
    control_motor(rc1, rc2)
    print "rc values:", rc1, rc2
    
    if rc1 == 0 and rc2 == 0:
        break

cap.release()
keyfile_rc.close()
vidfile.release()
