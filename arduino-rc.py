#!/usr/bin/python
import os
import time
import atexit
import serial
import math
import Adafruit_PCA9685

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
#    916/16666 * 4096 = 225 (right)
#   2110/16666 * 4096 = 519 (left)
#
# throttle pwm
#    876/16666 * 4096 = 215 (rew)
#   1496/16666 * 4096 = 368 (stop)
#   2070/16666 * 4096 = 509 (ffw)
 
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
pwm = Adafruit_PCA9685.PCA9685()

period = 0.05 # sec (=50ms)

# pulse: pwm length (us)
def calc_pwm(pulse):
        val = pulse * 4096 / 16666
        return val

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
        line = ser.readline().rstrip("\n\r")
        rc_inputs = line.split(' ')
        if len(rc_inputs) != 2:
                continue
        if int(rc_inputs[0]) == 0 or int(rc_inputs[1]) == 0:
                continue # there must be a timeout
        print "rc_str: {0}, rc_thr: {1}".format(rc_inputs[0], rc_inputs[1])
        
        # steering [0], throttle [1]
        steering_pwm = calc_pwm(int(rc_inputs[0]))
        throttle_pwm = calc_pwm(int(rc_inputs[1]))
        print "steering: {0}, throttle: {1}\n".format(steering_pwm, throttle_pwm)
        
        pwm.set_pwm(1, 0, steering_pwm)
        pwm.set_pwm(0, 0, throttle_pwm)

