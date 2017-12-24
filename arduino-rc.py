#!/usr/bin/python
import os
import time
import atexit
import serial
import math

# steering:
#   right: 916 us, center: 1516 us, left: 2110 us
#
# throttle:
#    rew: 876,   stop: 1476,  ffw: 2070

#
# PCA9658
#    resolution: 4096 (12bit counter)
#    freq = 60 Hz (16666 us period)

# pwm value calculation
#    length/period * 4096
#
# steering pwm
#    916/16666 * 4096 = 
# 
 
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
