#!/usr/bin/python
import os
import time
import atexit
import serial
import math
import cv2

# load configuration file
import cfg 

# arduino (rc input, sevor output)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=cfg.period/2)

# camera
cap = cv2.VideoCapture(0)
cap.set(3, cfg.cam_width) # TBD: error check on non supported resolution..
cap.set(4, cfg.cam_height)

# data files
fourcc = cv2.cv.CV_FOURCC(*'XVID')
vidfile = cv2.VideoWriter(cfg.data_video, fourcc, 15.0, (cfg.cam_width, cfg.cam_height))
keyfile = open(cfg.data_steer, 'w+')
keyfile.write("ts_micro,frame,wheel\n")

# stats
deadline_miss = 0

def turnOff(cap, vid, key):
        print "Closing the files..."
        cap.release()
        key.close()
        vid.release()
        print "Stats: deadline_miss={}".format(deadline_miss)
        
# linear map from X_range to Y_range
def map_range(x, X_min, X_max, Y_min, Y_max):
        X_range = X_max - X_min 
        Y_range = Y_max - Y_min 
        XY_ratio = float(X_range) / Y_range
        y = float((x - X_min)) / XY_ratio + Y_min 
        return y

# map steering pwm [916, 2110] to angle [-1, 1]
def pwm_to_angle(pulse):
        return map_range(pulse, cfg.str_min_pwm, cfg.str_max_pwm, -1, 1)

def g_tick():
        t = time.time()
        count = 0
        period = cfg.period
        while True:
                count += 1
                yield t + count*period

                
if __name__ == "__main__":
        # register exit callback
        atexit.register(turnOff, cap=cap, vid=vidfile, key=keyfile)

        # compute safe pwm cap
        thr_cap_ffw_pwm = int(cfg.thr_neu_pwm +
                              cfg.thr_cap_pct * (cfg.thr_max_pwm - cfg.thr_neu_pwm))
        thr_cap_rev_pwm = int(cfg.thr_neu_pwm -
                              cfg.thr_cap_pct * (cfg.thr_max_pwm - cfg.thr_neu_pwm))

        # initialize frame id 
        frame_id = 0

        # period tick
        g = g_tick()

        # main loop
        while (True):
                ts = time.time()
                next_ts = next(g)
                
                if next_ts > ts:
                        time.sleep(next_ts - ts)
                else:
                        print "deadline_miss: {}".format(deadline_miss)
                        deadline_miss += 1
                        continue
                ts = next_ts

                print "\nDBG: TS {:.3f}".format(ts)
                
                # 0. read a frame
                ret, frame = cap.read()

                # 1. get RC input
                ser.write("getrc\n")
                line = ser.readline().rstrip("\n\r")

                rc_inputs = line.split(' ')
                if len(rc_inputs) != 2:
                        continue
                if int(rc_inputs[0]) == 0 or int(rc_inputs[1]) == 0:
                        continue # there must be a timeout
                
                print "DBG: RC_INP {} {}".format(rc_inputs[0], rc_inputs[1])

                steering_pwm = int(rc_inputs[0])
                angle = pwm_to_angle(steering_pwm) # angle: -1 .. 1
        
                throttle_pwm = int(rc_inputs[1])
                throttle_pwm = min(throttle_pwm, thr_cap_ffw_pwm)
                throttle_pwm = max(throttle_pwm, thr_cap_rev_pwm)

                # 2. control: steering [0], throttle [1]
                cmd = "setpwm {0} {1}\n".format(steering_pwm, throttle_pwm)
                ser.write(cmd)
                print "DBG: SETPWM {} {}".format(steering_pwm, throttle_pwm)

                # 3. record data
                if throttle_pwm == thr_cap_ffw_pwm:
                        # increase frame_id
                        frame_id = frame_id + 1
                        
                        # write input (angle)
                        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
                        keyfile.write(str)
                        print "LOG: TS {:.3f},{},{:.3f}".format(ts, frame_id, angle)
                
                        # write video stream
                        vidfile.write(frame)


