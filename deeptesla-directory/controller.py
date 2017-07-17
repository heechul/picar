#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import os
import time
import atexit

import cv2
import csv

import preprocess
import model
import tensorflow as tf
import local_common as cm
import params
import os
import time

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOff():
	stop()#Stop the car if it is moving
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
	os.system("killall -HUP mencoder")
	
atexit.register(turnOff)

steeringMotor = mh.getMotor(1)
speedMotor = mh.getMotor(3)

MAX_STEERING = 50 # safe margin.
MAX_SPEED = 1500  # pwm  1500us/20ms = max full throttle
NEU_SPEED = 1270  # pwm  1270us/20ms = stop
MIN_SPEED = 950   # pwm   950us/20ms = reverse full throttle

currentSpeed = NEU_SPEED
angleArray = [] #Holds the steering angles collected while a video is recording
angleVal = 0 #Angle value that is added to angleArray, should be between -1 and 1
setNum = 1 #Holds the dataset number
while(os.path.exists("datasets/dataset%i" % setNum)): #While a dataset of the current number exists
    setNum+=1 #Increase the dataset number

# set the speed to start, from 0 (off) to 255 (max speed)
steeringMotor.setSpeed(127)
steeringMotor.run(Adafruit_MotorHAT.FORWARD);
# turn on motor
steeringMotor.run(Adafruit_MotorHAT.RELEASE);

# set the speed to start, from 0 (off) to 255 (max speed)
speedMotor.setSpeed(127)
speedMotor.run(Adafruit_MotorHAT.FORWARD);
# turn on motor
speedMotor.run(Adafruit_MotorHAT.RELEASE);


def right(speed, delay):
        steeringMotor.run(Adafruit_MotorHAT.FORWARD)
        steeringMotor.setSpeed(speed)
        #print "left" 
        time.sleep(delay)
        steeringMotor.run(Adafruit_MotorHAT.RELEASE);

def left(speed, delay):
        steeringMotor.run(Adafruit_MotorHAT.BACKWARD)
        steeringMotor.setSpeed(speed)
        #print "right"
        time.sleep(delay)
        steeringMotor.run(Adafruit_MotorHAT.RELEASE);

def ffw(inc):
        global currentSpeed
        currentSpeed = currentSpeed + inc
        if currentSpeed > MAX_SPEED:
                currentSpeed = MAX_SPEED
	os.system("echo 0=%dus > /dev/servoblaster" % currentSpeed)
	print "accel:", currentSpeed
def rew(dec):
        global currentSpeed
	if currentSpeed == NEU_SPEED:
		currentSpeed = 1250
	else:
        	currentSpeed = currentSpeed - dec
        if currentSpeed < MIN_SPEED:
                currentSpeed = MIN_SPEED
	os.system("echo 0=%dus > /dev/servoblaster" % currentSpeed)
        print "slowdown: ", currentSpeed
def stop():
        global currentSpeed
        currentSpeed = NEU_SPEED
        print "stop"
	os.system("echo 0=%dus > /dev/servoblaster" % currentSpeed)

#Stop source warnings from appearing
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

#Dataset that will be used for predicting the angle
dataset_id = 12

#Open the model
sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

#Determine if the camera (1) or a video (0) is used for predicting the angle
useCamera = 1

#Start the car
ffw(40)

#If using the camera
if useCamera:
    cap = cv2.VideoCapture(0) #Capture the input from the camera
	
    tempAngle = 0 #Hold the angle

    #Get the predicted angle from the model for each frame and have the car turn accordingly
    while(True):
        startTime = time.time()
        ret, img = cap.read()
        if not ret: #If there isn't a frame
            stop() #Stop the car
        img = preprocess.preprocess(img) #Process the image
        deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
        deg = round(deg * 8) / 8 #Round the angle to the nearest eighth
        delay = abs(.1 * ((deg - tempAngle) / .125)) #Calculate the turn delay
        if deg < tempAngle: #left
            right(MAX_STEERING, delay)
        elif deg > tempAngle: #right
            left(MAX_STEERING, delay)
        print deg , " " , tempAngle , " " , delay , " " , round((time.time() - startTime) * 1000)
        tempAngle = deg #Update the angle
else:
    totalTime = time.time()

    #Open the video and get the number of frames
    vid_path = params.data_dir + "/dataset{0}/out-mencoder.avi".format(dataset_id)
    assert os.path.isfile(vid_path)
    frame_count = cm.frame_count(vid_path)
    cap = cv2.VideoCapture(vid_path)
	
    print "Using video from" + vid_path + " which has a total of {} frames".format(frame_count)

    tempAngle = 0 #Hold the angle

    #Get the predicted angle from the model for each frame and have the car turn accordingly
    for frame_id in xrange(frame_count):
        startTime = time.time()
        ret, img = cap.read()
        if not ret: #If there isn't a frame
            stop() #Stop the car
        img = preprocess.preprocess(img) #Process the image
        deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
        deg = (round(deg * 8) / 8) #Round the angle to the nearest eighth
        delay = abs(.1 * ((deg - tempAngle) / .125)) #Calculate the turn delay
        if deg < tempAngle: #right
            right(MAX_STEERING, delay)
        elif deg > tempAngle: #left
            left(MAX_STEERING, delay)
        print deg , " " , delay , " in " , round((time.time() - startTime) * 1000) , " ms"
        tempAngle = deg #Update the angle

stop() #Stop the car when done if necessary
print round((time.time() - totalTime))