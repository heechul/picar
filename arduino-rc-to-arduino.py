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
        ser.write("getrc\n")
        ts_start = time.time()
        line = ser.readline().rstrip("\n\r")
        ts_end   = time.time()
        print "line:{0} {1} ms".format(line, (ts_end - ts_start)*1000)
        rc_inputs = line.split(' ')
        if len(rc_inputs) != 2:
                continue
        if int(rc_inputs[0]) == 0 or int(rc_inputs[1]) == 0:
                continue # there must be a timeout
        print "rc_thr: {0}, rc_str: {1}".format(rc_inputs[0], rc_inputs[1])
        
        # steering [0], throttle [1]
        cmd = "setpwm {0} {1}\n".format(rc_inputs[0], rc_inputs[1])
        print cmd
        ser.write(cmd)

