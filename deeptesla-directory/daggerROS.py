#!/usr/bin/python
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

import rospy
from geometry_msgs.msg import Twist

#Stop source warnings from appearing
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

targetSpeed = 0 #Speed that the car should travel

# recommended for auto-disabling motors on shutdown!
def turnOff():
	#Create a twist message that stops the car and publish it
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
	
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

key = ''

#Create a thread for getting input from the user
def getExpertInput():
	global key
	while(True):
		key = read_single_keypress()
		print key
		if key == 's':
			break
			
#Create a node that publishes to cmd_vel
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('dagger_pub')

#Open the model
sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

cap = cv2.VideoCapture('http://devboard-picar-wifi.ittc.ku.edu:8080/?action=stream') #Capture the input from the stream
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
		turnOff()
	frameArray.append(img)
	img = preprocess.preprocess(img) #Process the image
	deg_m = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
	deg_m = round(deg_m * 8) / 8 #Round the angle to the nearest eighth
	deg_m = deg_m - tempAngle
	degMArray.append(deg_m)
	
	deg_h = 0 #Initialize human angle to 0
	turnVar = .5
	
	#If there was input from the expert
	if not key == '':		
		if key == 'j':
			deg_h = .125
			turnVar = 1
		elif key == 'k':
			deg_h = -.125
			turnVar = -1
		elif key == 'a':
			targetSpeed += .05
		elif key == 'z':
			targetSpeed -= .05
		elif key == 's':
			break
	
	degHArray.append(deg_h) #Add the expert angle to the appropriate array
		
	#If there wasn't any expert input or the machine angle should be used
	if key == '' or random.random() > .3:
		deg = deg_m #Choose the machine angle
	else: #The expert angle should be used
		deg = deg_h #Choose the expert angle
	
	if deg < 0: #If the degree is negative
		turnVar = turnVar * -1 #Change the turn value so that the car turns right
		deg = -.125
	else:
		deg = .125
		
	#Create a twist message that determines if a turn is necessary and publish it
	twist = Twist()
	twist.linear.x = targetSpeed; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turnVar
	pub.publish(twist)
	
	#Send a second message to stop the wheel turning
	twist.angular.z = 0
	pub.publish(twist)
		
	key = '' #Reset the key to empty
		
	degChosen.append(deg) #Add the chosen angle to the appropriate array
	
	print round((time.time() - startTime) * 1000) #Print the total time it took for that frame to be processed
	tempAngle += deg #Update the angle
	
	actualDeg.append(tempAngle)
	
turnOff()
expertInput.join() #Have the expert input thread, which should be done, join back to the main thread
	
with open("../datasets/dataset_dagger/data.csv", 'wb') as file: #Open the csv file
	writer = csv.writer(file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL) #Create a writer
	writer.writerow(['frame', 'machine angle', 'expert angle', 'chosen angle', 'angle']) #Write the first row that labels the columns
	for i in range (len(degChosen)): #For each frame
		writer.writerow([i, degMArray[i], degHArray[i], degChosen[i], actualDeg[i]]) #Write the angles to the csv file
		
#Remove the last frame in the case that there aren't enought angles recorded
if not len(frameArray) == len(actualDeg):
	frameArray.pop()

#Create a video using the frames collected
clip = ImageSequenceClip(frameArray, fps=15) 
clip.write_videofile('../datasets/dataset_dagger/dagger.mp4') #