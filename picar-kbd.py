#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import os
import time
import atexit

import threading
import cv2
import csv

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOff():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
	os.system("killall -HUP mencoder")
	
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
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
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

################################# DC motor test!
steeringMotor = mh.getMotor(1)
speedMotor = mh.getMotor(3)

MAX_STEERING = 50 # safe margin.
MAX_SPEED = 1500  # pwm  1500us/20ms = max full throttle
NEU_SPEED = 1270  # pwm  1270us/20ms = stop
MIN_SPEED = 950   # pwm   950us/20ms = reverse full throttle

currentSpeed = NEU_SPEED
angleArray = [] #Holds the steering angles collected while a video is recording
angleVal = 0 #Angle value that is added to angleArray, should be between -1 and 1

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
        global angleVal
        steeringMotor.run(Adafruit_MotorHAT.FORWARD)
        steeringMotor.setSpeed(speed)
        print "left" 
        if(angleVal > -.9):	#If the angle can be decreased without going below -1
            angleVal -= .125 #Decrease the angle by .125
        print(angleVal) #Print the angle value
        time.sleep(delay)
        steeringMotor.run(Adafruit_MotorHAT.RELEASE);

def left(speed, delay):
        global angleVal
        steeringMotor.run(Adafruit_MotorHAT.BACKWARD)
        steeringMotor.setSpeed(speed)
        print "right"
        if(angleVal < .9): #If the angle ca be increased without going above 1
            angleVal += .125 #Increase the angle by .125
        print(angleVal) #Print the angle value
        time.sleep(delay)
        steeringMotor.run(Adafruit_MotorHAT.RELEASE);

def ffw(inc):
        global currentSpeed
        currentSpeed = currentSpeed + inc
        if currentSpeed > MAX_SPEED:
                currentSpeed = MAX_SPEED
	os.system("echo 0=%dus > /dev/servoblaster" % currentSpeed)
	print "accel:", currentSpeed
def stop():
        global currentSpeed
        currentSpeed = NEU_SPEED
        print "stop"
	os.system("echo 0=%dus > /dev/servoblaster" % currentSpeed)

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
        
recCmd = 'start'

def getData():
    global angleVal, angleArray
    while (True): #Continually get the steering angle till prompted to stop
        if recCmd == 'start': #If the video has stop recording
            break #Stop collecting the steering angle
        angleArray.append(angleVal) #Get the steering angle
        time.sleep(.033) #Delay the thread so that angle values aren't continually added to the array

def parseData():
    global angleArray
    video = cv2.VideoCapture('out-mencoder.avi') #Open the video
    framesLeft,image = video.read() #Read the current frame
    frameNum = 0 #Hold the frame number
    while framesLeft: #While there are still frames in the video
        cv2.imwrite("data/png/Frame%d.png" % frameNum, image) #Save the frame as a png image
        frameNum += 1 #Increase the frame number by 1
        framesLeft,image = video.read() #Read the current frame
    with open("data/data.csv", 'wb') as file: #Open the csv file
        writer = csv.writer(file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL) #Create a writer
        for i in range (frameNum): #For each frame
            writer.writerow([os.getcwd() + '/data/png/Frame%d.png' % i, angleArray[i]]) #Write the path to the frame and the steering angle to the csv file
    angleArray = [] #Empty the angle array for the next time data is gathered
    print("Data has been collected") #Tell the user the data has been saved
    print("%i frames have been saved" % frameNum)
			
#Create a thread for collecting the steering angle
data_thread = threading.Thread(target=getData)

def record_video():
	global recCmd
	if recCmd == 'start':
		os.system("mencoder tv:// -tv driver=v4l2:width=640:height=480:device=/dev/video0 -nosound -ovc lavc -o out-mencoder.avi &")
		recCmd = 'stop'
		time.sleep(2) #Wait for mencoder to start recording, two seconds seems to be an appropriate time to wait
		data_thread.start() #Start collecting the steering angle
	else:
		os.system("killall -HUP mencoder")
		recCmd = 'start'
		data_thread.join() #Wait for the getData method to finish
		print(len(angleArray))
		parseData() #Go through and save the data collected

while (True):
        ch = read_single_keypress()
        print "ch = ", ch 
        if ch == 'j':
                left(MAX_STEERING, 0.10)
        elif ch == 'k':
                right(MAX_STEERING, 0.10)
        elif ch == 'a':
                ffw(10)
        elif ch == 's':
                print(len(angleArray))
                stop()
        elif ch == 'z':
                rew(10)
        elif ch == 'q':
                break
	elif ch == 'r':
		record_video()
