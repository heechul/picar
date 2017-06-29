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

#If a folder for holding the datasets doesn't exist
if not(os.path.exists('datasets')):
	os.makedirs('datasets') #Create the datasets directory

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

#Record the steering angle of the car while a video is recording
def getData():
    global angleVal, angleArray
    while (True): #Continually get the steering angle till prompted to stop
        if recCmd == 'start': #If the video has stop recording
            angleArray.append(angleVal) #Get the angle for the last frame
            break #Stop collecting the steering angle
        angleArray.append(angleVal) #Get the steering angle
        time.sleep(.033) #Delay the thread so that angle values aren't continually added to the array

#Find the current dataset number and create the necessary directories and files for it
def makeRunDir():
    global setNum #Get the dataset number
    if not (os.path.exists("datasets/dataset%i" % setNum)): #If a directory for the dataset doesn't already exist
        os.makedirs("datasets/dataset%i" % setNum, 0755) #Create a directory for the new dataset
        os.makedirs("datasets/dataset%i/png" % setNum, 0755) #Create a directory for the image files
        file = open("datasets/dataset%i/data.csv" % setNum, "w").close() #Create a csv file for saving the data

#Go through all of the collected data and save it
def parseData():
    global angleArray, setNum
    video = cv2.VideoCapture('datasets/dataset%i/out-mencoder.avi' % setNum) #Open the video
    framesLeft,image = video.read() #Read the current frame
    frameNum = 0 #Hold the frame number
    while framesLeft: #While there are still frames in the video
        cv2.imwrite("./datasets/dataset%i/png/Frame%d.png" % (setNum, frameNum), image) #Save the frame as a png image
        frameNum += 1 #Increase the frame number by 1
        framesLeft,image = video.read() #Read the current frame
    with open("./datasets/dataset%i/data.csv" % setNum, 'wb') as file: #Open the csv file
        writer = csv.writer(file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL) #Create a writer
        writer.writerow(['image', 'angle']) #Write the first row that labels the columns
        for i in range (frameNum): #For each frame
            writer.writerow([os.getcwd() + '/datasets/dataset%i/png/Frame%d.png' % (setNum, i), angleArray[i]]) #Write the path to the frame and the steering angle to the csv file
    angleArray = [] #Empty the angle array for the next time data is gathered
    print("%i frames have been saved" % frameNum)
    print("Data has been collected in datasets/dataset%i" % setNum) #Tell the user the data has been saved
			
#Create a thread for collecting the steering angle
data_thread = threading.Thread(target=getData)

def record_video():
	global recCmd, data_thread, setNum
	if recCmd == 'start':
		makeRunDir()
		os.system("mencoder tv:// -tv driver=v4l2:width=320:height=240:device=/dev/video0 -fps 15 -nosound -ovc lavc -o datasets/dataset%i/out-mencoder.avi &" % setNum)
		recCmd = 'stop'
		time.sleep(2.09) #Wait for mencoder to start recording, a little over two seconds seems to be an appropriate time to wait
		data_thread.start() #Start collecting the steering angle
	else:
		stop()
		os.system("killall -HUP mencoder")
		recCmd = 'start'
		data_thread.join() #Wait for the getData method to finish
		time.sleep(.5) #Wait so that the number of angles is printed after mencoder displays all video information
		print("%i angles were collected" % len(angleArray)) #Print the number of angles collected to ensure that it's close to the number of frames in the video
		parseData() #Go through and save the data collected
		data_thread = threading.Thread(target=getData) #Recreate the data thread, allows for multiple datasets to be captured without having to restart picar-kbd.py
		setNum += 1 #Move to the next dataset

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
                stop()
        elif ch == 'z':
                rew(10)
        elif ch == 'q':
                stop()
                break
	elif ch == 'r':
		record_video()
	elif ch == 't':
		angleVal = 0 #Reset the steering angle to 0 in the case that the wheels need to be centered after picar-kbd.py is started
		print("Angle reset to %d" % angleVal) #Print out the steering angle so the user knows it was reset to 0
