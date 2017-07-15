#!/usr/bin/python
import os
import time
import atexit
import serial
import cv2
import math
import numpy as np
import pygame
from pololu_drv8835_rpi import motors, MAX_SPEED

motors.setSpeeds(0, 0)
cap = cv2.VideoCapture(0)
# cap.set(3,320)
# cap.set(4,240)

# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 15.0, (640,480))
keyfile = open('out-key.csv', 'w+')
keyfile_btn = open('out-key-btn.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")
keyfile_btn.write("ts_micro,frame,btn,speed\n")
rec_start_time = 0
cur_speed = MAX_SPEED   # 0...MAX_SPEED
cur_steer = MAX_SPEED

def stop():
        motors.motor2.setSpeed(0)
        motors.motor1.setSpeed(0)        

# 
def ffw():
        global cur_speed
        motors.motor2.setSpeed(cur_speed)
def rew():
        global cur_speed        
        motors.motor2.setSpeed(-cur_speed)
# W - throttle speed up         
def throttle_up():
        global cur_speed        
        cur_speed = min(MAX_SPEED, cur_speed + MAX_SPEED/10)
        print "cur_speed:", cur_speed
# X - throttle speed down 
def throttle_down():
        global cur_speed        
        cur_speed = max(0, cur_speed - MAX_SPEED/10)
        print "cur_speed:", cur_speed        
        
def left():
        global cur_steer
        motors.motor1.setSpeed(cur_steer)
def right():
        global cur_steer
        motors.motor1.setSpeed(-cur_steer)
        
# E - throttle speed up         
def steer_up():
        global cur_steer
        cur_steer = min(MAX_SPEED, cur_steer + MAX_SPEED/10)
        print "cur_steer:", cur_steer        
# C - throttle speed down 
def steer_down():
        global cur_steer        
        cur_steer = max(0, cur_steer - MAX_SPEED/10)
        print "cur_steer:", cur_steer         
        
def degree2rad(deg):
        return deg * math.pi / 180.0

def turnOff():
        stop()

atexit.register(turnOff)

view_video = False
frame_id = 0
null_frame = np.zeros((160,120,3), np.uint8)
cv2.imshow('frame', null_frame)

angle = 0.0
btn   = ''

while (True):
        # read a frame
        ret, frame = cap.read()
        ts = int(time.time() * 1000000)

        if view_video == True:
                cv2.imshow('frame', frame)

        ch = cv2.waitKey(10) & 0xFF # wait 10ms

        if ch == ord('j'):
                left()
                print "left: ", angle
                angle = max(-30, angle - 12)
                btn   = ch
        elif ch == ord('l'):
                right()
                print "right: ", angle
                angle = min(+30, angle + 12)
                btn   = ch                
        elif ch == ord('i'):
                ffw()
                print "accel:", cur_speed
        elif ch == ord('k'):
                stop()
                print "stop"                
                btn   = ch
        elif ch == ord(','):
                rew()
                print "reverse:", cur_speed
        elif ch == ord('a'):
                throttle_up()
        elif ch == ord('z'):
                throttle_down()
        elif ch == ord('e'):
                steer_up()
        elif ch == ord('c'):
                steer_down()
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
	else: # if no key, stop angle motor
                motors.motor1.setSpeed(0)          
                        
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

	        print ts, frame_id, angle, btn

cap.release()
keyfile.close()
keyfile_btn.close()
vidfile.release()
