#!/usr/bin/python
import os
import time
import atexit
import serial
import math

# steering:
#   left: 916, center: 1516, right: 2110
#
# throttle:
#    ffw: 906,   stop: 1476,  back: 2070

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
period = 0.05 # sec (=50ms)

def g_tick():
        t = time.time()
        count = 0
        while True:
                count += 1
                yield max(t + count*period - time.time(),0)
        
g = g_tick()

while (True):
        time.sleep(next(g))
        ts = time.time()        
        ser.write(b'getrc\n')
        line = ser.readline()
        print(line)
