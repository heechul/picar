#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from moviepy.editor import ImageSequenceClip

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
import random
import numpy as np

import threading

import Image
import ImageDraw

#Stop source warnings from appearing
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOff():
	stop()#Stop the car if it is moving
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
	
atexit.register(turnOff)

def read_single_keypress():
    """Waits for a single keypress on stdin.

    This is a silly function to call if you need to do it a lot because it has
    to store stdin's current setup, setup stdin for reading single keystrokes
    then read the single keystroke then revert stdin back after reading the
    keystroke.

    Returns the character of the key that was pressed (zero on
    KeyboardInterrupt which can happen when a signal gets handled)

    """
    import termios, fcntl, sys, os
    fd = sys.stdin.fileno()
    # save old state
    flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
    attrs_save = termios.tcgetattr(fd)
    # make raw - the way to do this comes from the termios(3) man page.
    attrs = list(attrs_save) # copy the stored version to update
    # iflag
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK 
                  | termios.ISTRIP | termios.INLCR | termios. IGNCR 
                  | termios.ICRNL | termios.IXON )
    # oflag
    attrs[1] &= ~termios.OPOST
    # cflag
    attrs[2] &= ~(termios.CSIZE | termios. PARENB)
    attrs[2] |= termios.CS8
    # lflag
    attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON
                  | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # turn off non-blocking
    #fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
    # read a single keystroke
    try:
        ret = sys.stdin.read(1) # returns a single character
    except KeyboardInterrupt: 
        ret = 0
    finally:
        # restore old state
        termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
    return ret

steeringMotor = mh.getMotor(1)
speedMotor = mh.getMotor(3)

MAX_STEERING = 50 # safe margin.
MAX_SPEED = 1500  # pwm  1500us/20ms = max full throttle
NEU_SPEED = 1270  # pwm  1270us/20ms = stop
MIN_SPEED = 950   # pwm   950us/20ms = reverse full throttle

currentSpeed = NEU_SPEED

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

key = '' #Holds the key inputs of the user/expert

#Create a thread for getting input from the user
def getExpertInput():
	global key
	while(True):
		key = read_single_keypress()
		print key
		if key == 's':
			break
		elif key == 'a': #accelerate
			ffw(10)
		elif key == 'z': #decelerate
			rew(10)

#Open the model
sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

#Start the car
ffw(40)

cap = cv2.VideoCapture(0) #Capture the input from the camera
cap.set(3,320) #Change the input to be 320x240
cap.set(4,240)

degHArray = [] #Holds the angles of the expert
degMArray = [] #Holds the angles of the DNN
degChosen = [] #Holds the degrees chosen
actualDeg = [] #Holds the degrees of the cars
frameArray = [] #Holds the frames so that they can be used to create a video

tempAngle = 0 #Hold the angle

#Create the thread for getting expert input and start it
expertInput = threading.Thread(target=getExpertInput) 
expertInput.start()

#Get the predicted angle from the model for each frame and have the car turn accordingly
while(True):
	startTime = time.time()
	ret, img = cap.read()
	if not ret: #If there isn't a frame
		stop() #Stop the car
	frameArray.append(img)
	img = preprocess.preprocess(img) #Process the image
	deg_m = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
	deg_m = round(deg_m * 8) / 8 #Round the angle to the nearest eighth
	if deg_m < tempAngle:
		deg_m = -.125
	elif deg_m > tempAngle:
		deg_m = .125
	elif deg_m == tempAngle:
		deg_m = 0
	degMArray.append(deg_m)
	
	deg_h = 0 #Initialize human angle to 0
	
	#If there was input from the expert
	if not key == '':		
		if key == 'j': #left
			deg_h = .125
		elif key == 'k': #right
			deg_h = -.125
		elif key == 's': #stop
			break
			
	degHArray.append(deg_h) #Add the expert angle to the appropriate array
		
	#If there wasn't any expert input or the machine angle should be used
	if key == '' or random.random() > .5:
		deg = deg_m #Choose the machine angle
	else: #The expert angle should be used
		deg = deg_h #Choose the expert angle
		
	key = '' #Reset the key to empty
	
	#Turn left or right if necessary based on chosen angle
	if deg < 0: #right
		right(MAX_STEERING, .1)
	elif deg > 0: #left
		left(MAX_STEERING, .1)
		
	degChosen.append(deg) #Add the chosen angle to the appropriate array
	
	print round((time.time() - startTime) * 1000) #Print the total time it took for that frame to be processed
	tempAngle += deg #Update the angle
	if tempAngle > 1:
		tempAngle = 1
	elif tempAngle < -1:
		tempAngle = -1
	
	actualDeg.append(tempAngle) #Add the actual angle to the appropriate array
	
stop() #Stop the car
expertInput.join() #Have the expert input thread, which should be done, join back to the main thread
	
with open("../datasets/dataset_dagger/data.csv", 'wb') as file: #Open the csv file
	writer = csv.writer(file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL) #Create a writer
	writer.writerow(['frame', 'machine angle', 'expert angle', 'chosen angle', 'angle']) #Write the first row that labels the columns
	for i in range (len(degChosen)): #For each frame
		writer.writerow([i, degMArray[i], degHArray[i], degChosen[i], actualDeg[i]]) #Write the angles to the csv file
		
#Remove the last frame in the case that there aren't enought angles recorded
if not len(frameArray) == len(actualDeg):
	frameArray.pop()
	
textColor = (255,255,255) #Declare the color for the new image background
bgColor = (0,0,0) #Declare the color for the new image text

#Add the frame number and state variable value to each frame
for i in xrange(len(frameArray)):
	newImage = Image.new('RGBA', (100, 20), bgColor) #Create blank canvas for metrics
	drawer = ImageDraw.Draw(newImage) #Create ImageDraw for drawing the metrics
	drawer.text((0, 0), "Frame #{}".format(i), fill=textColor) #Print the frame number
	drawer.text((0, 10), "State Var:{}".format(actualDeg[i]), fill=textColor) #Print the value of the state variable
	newImage = cv2.cvtColor(np.array(newImage), cv2.COLOR_BGR2RGBA) #Change the new image to cv2 format
	frameArray[i] = cm.overlay_image(frameArray[i], newImage, x_offset = 0, y_offset = 0) #Add the new image to the frame

#Create a video using the frames collected
clip = ImageSequenceClip(frameArray, fps=15) 
clip.write_videofile('../datasets/dataset_dagger/dagger.mp4') #