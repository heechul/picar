#!/usr/bin/python
import os
import time
import atexit
import serial
import cv2

def turnOff():
	os.system("killall -HUP mencoder")

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

atexit.register(turnOff)

ser = serial.Serial('/dev/ttyACM0', 115200)
keyfile = open('out-key.csv', 'w+')
recstat = 'idle'
rec_start_time = 0

def right():
        ser.write(b'r\n')
        ts = time.time() - rec_start_time
        s="{} right\n".format(ts)
        if rec_start_time > 0:        
                keyfile.write(s)
        print s
        
def left():
        ser.write(b'l\n')
        ts = time.time() - rec_start_time        
        s="{} left\n".format(ts)
        if rec_start_time > 0:
                keyfile.write(s)
        print s
        
def center():
        ser.write(b'c\n')
        ts = time.time() - rec_start_time                
        s="{} center\n".format(ts)
        if rec_start_time > 0:        
                keyfile.write(s)
        print s
        
def ffw():
        ser.write(b'f\n')
        ts = time.time() - rec_start_time                        
        s="{} forward\n".format(ts)
        if rec_start_time > 0:        
                keyfile.write(s)
        print s
        
def rew():
        ser.write(b'b\n')
        ts = time.time() - rec_start_time        
        s="{} backward\n".format(ts)
        if rec_start_time > 0:        
                keyfile.write(s)
        print s
        
def stop():
        ser.write(b's\n')
        ts = time.time() - rec_start_time                                
        s="{} stop\n".format(ts)
        keyfile.write(s)
        print s
        
def record_video():
	global recstat, rec_start_time
	if recstat == 'idle':
		os.system("mencoder tv:// -tv driver=v4l2:width=320:height=240:device=/dev/video0 -nosound -ovc lavc -o out-mencoder.avi 2> /dev/null &")
		recstat = 'recording'
                rec_start_time = time.time()
	else:
		os.system("killall -HUP mencoder")
		recstat = 'idle'
                rec_start_time = 0

while (True):
        ch = read_single_keypress()
        print "ch = ", ch 
        if ch == 'j':
                left()
        elif ch == 'k':
                center()
        elif ch == 'l':
                right()
        elif ch == 'a':
                ffw()
        elif ch == 's':
                stop()
        elif ch == 'z':
                rew()
        elif ch == 'q':
                break
	elif ch == 'r':
		record_video()
