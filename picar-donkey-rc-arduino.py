#!/usr/bin/python
import os
import time
import atexit
import serial
import math
import cv2

# steering:
#   right: 916 us, center: 1516 us, left: 2110 us
#
# throttle:
#    rew: 876,   stop: 1476,  ffw: 2070

str_left_pwm = 940
str_right_pwm = 2140

thr_max_pwm = 2070
thr_neu_pwm = 1476
thr_cap_pct = 0.20  # 20% max
thr_cap_pwm = int(thr_neu_pwm + thr_cap_pct * (thr_max_pwm - thr_neu_pwm))
thr_cap_pwm_rev = int(thr_neu_pwm - thr_cap_pct * (thr_max_pwm - thr_neu_pwm))


ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
period = 0.05 # sec (=50ms)

cap = cv2.VideoCapture(0)
cap.set(3,640) # TBD: error check on non supported resolution..
cap.set(4,480)

fourcc = cv2.cv.CV_FOURCC(*'XVID')

vidfile = cv2.VideoWriter('out-video.avi', fourcc, 15.0, (640,480))
keyfile = open('out-key.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")
frame_id = 0

def turnOff():
        print "Closing the files..."
        cap.release()
        keyfile.close()
        vidfile.release()
        
atexit.register(turnOff)

# linear map from X_range to Y_range
def map_range(x, X_min, X_max, Y_min, Y_max):
        X_range = X_max - X_min 
        Y_range = Y_max - Y_min 
        XY_ratio = float(X_range) / Y_range
        y = float((x - X_min)) / XY_ratio + Y_min 
        return y

# map steering pwm [916, 2110] to angle [-1, 1]
def pwm_to_angle(pulse):
        return map_range(pulse, str_left_pwm, str_right_pwm, -1, 1)


# period tick
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

        # 0. read a frame
        ret, frame = cap.read()

        # 1. get RC input
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
        print "rc_str: {0}, rc_thr: {1}".format(rc_inputs[0], rc_inputs[1])

        steering_pwm = int(rc_inputs[0])
        throttle_pwm = int(rc_inputs[1])
        throttle_pwm = min(throttle_pwm, thr_cap_pwm)
        throttle_pwm = max(throttle_pwm, thr_cap_pwm_rev)

        # 2. control: steering [0], throttle [1]
        cmd = "setpwm {0} {1}\n".format(steering_pwm, throttle_pwm)
        print cmd
        ser.write(cmd)

        # 3. record data
        if throttle_pwm == thr_cap_pwm:
                # increase frame_id
                frame_id = frame_id + 1

                # convert steering pwm to angle
                angle = pwm_to_angle(steering_pwm)

                # write input (angle)
                str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
                keyfile.write(str)
                print "DBG:", str
                
                # write video stream
                vidfile.write(frame)

print "Finished.."
turnOff()
