#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import os
import time
import atexit

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
        print "left"        
        time.sleep(delay)
        steeringMotor.run(Adafruit_MotorHAT.RELEASE);

def left(speed, delay):
        steeringMotor.run(Adafruit_MotorHAT.BACKWARD)
        steeringMotor.setSpeed(speed)
        print "right"
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
def record_video():
	global recCmd
	if recCmd == 'start':
		os.system("mencoder tv:// -tv driver=v4l2:width=640:height=480:device=/dev/video0 -nosound -ovc lavc -o out-mencoder.avi &")
		recCmd = 'stop'
	else:
		os.system("killall -HUP mencoder")
		recCmd = 'start'

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
                break
	elif ch == 'r':
		record_video()
