#!/usr/bin/python

import os
import time
import atexit
import serial
import math
import cv2
import numpy as np

# load configuration file
import cfg
period = 1.0/cfg.cam_fps    

# feature flags.
use_debug = False
use_dnn   = True

def DEBUG(text):
    if use_debug == True:
        print text

def turn_off(cap, vid, key, stat):
    print "Closing the files..."
    cap.release()
    key.close()
    vid.release()

    print "Control loop statistics"
    print "======================="
    print "min/avg/99pct/max (sec): {:.3f} {:.3f} {:.3f} {:.3f}".format(
        np.min(stat), np.mean(stat), np.percentile(stat, 99), np.max(stat))
    dmiss_cnt = sum(i > period for i in stat)
    print "deadline miss(es): {}/{} {:.1f}%".format(dmiss_cnt, len(stat),
                                                float(dmiss_cnt)/len(stat)*100)

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

def angle_to_pwm(angle):
    return map_range(angle, -1, 1, cfg.str_min_pwm, cfg.str_max_pwm)

if __name__ == "__main__":
    # load DNN if needed.
    if use_dnn:
        print "Load TF"
        import tensorflow as tf
        print "Load deeppicar model"
        import model
        print "Load support modules.."
        import params
        import local_common as cm
        import preprocess
        import numpy as np

        print "Open session"
        sess = tf.InteractiveSession()
        saver = tf.train.Saver()
        model_name = 'model.ckpt'
        model_path = cm.jn(params.save_dir, model_name)
        print "Load model file into the TF session"
        saver.restore(sess, model_path)
        print "Done..."


    # arduino (rc input, sevor output)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    
    # camera
    cap = cv2.VideoCapture(0)
    cap.set(3, cfg.cam_width) # TBD: error check on non supported resolution..
    cap.set(4, cfg.cam_height)
    cap.set(5, cfg.cam_fps) # this determines the control loop frequency.
        
    # data files
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    vidfile = cv2.VideoWriter(cfg.data_video, fourcc,
                              cfg.cam_fps, (cfg.cam_width, cfg.cam_height))
    keyfile = open(cfg.data_steer, 'w+')
    keyfile.write("ts_micro,frame,wheel\n")
    
    # stats
    dstat = []
    
    # register exit callback
    atexit.register(turn_off, cap=cap, vid=vidfile, key=keyfile, stat=dstat)

    # compute safe pwm cap
    thr_cap_ffw_pwm = int(cfg.thr_neu_pwm +
                          cfg.thr_cap_pct * (cfg.thr_max_pwm - cfg.thr_neu_pwm))
    thr_cap_rev_pwm = int(cfg.thr_neu_pwm -
                          cfg.thr_cap_pct * (cfg.thr_max_pwm - cfg.thr_neu_pwm))

    print "INFO: to start recording, set the RC throttle to the max."

    # grab the first image.
    ret, frame = cap.read()
    ts = init_ts = time.time()
    frame_id = 0

    # main loop
    while (True):
        # 1. get RC input
        ser.write("getrc\n")
        line = ser.readline().rstrip("\n\r")

        rc_inputs = line.split(' ')
        if len(rc_inputs) != 2:
            continue
        if int(rc_inputs[0]) == 0 or int(rc_inputs[1]) == 0:
            continue # there must be a timeout
                
        DEBUG("{:.3f} RC_INP {} {}".format(ts-init_ts, rc_inputs[0], rc_inputs[1]))

        steering_pwm = int(rc_inputs[0])
        angle_rc = pwm_to_angle(steering_pwm) # angle: -1 .. 1

        # 1. get machine input (if applicable)
        if use_dnn:
            # image pre-processing.
            img = preprocess.preprocess(frame)

            # DNN inferencing.
            angle_dnn = model.y.eval(feed_dict={model.x: [img],
                                                model.keep_prob: 1.0})[0][0]
                        
            if angle_rc > -0.05 and angle_rc < 0.05:
                angle = angle_dnn                 
                steering_pwm = int(angle_to_pwm(angle))
            else:
                print "RC override: rc={:.3f} dnn={:.3f}".format(angle_rc, angle_dnn)
                angle = angle_rc
        else:
            angle = angle_rc
        
        throttle_pwm = int(rc_inputs[1])
        throttle_pwm = min(throttle_pwm, thr_cap_ffw_pwm)
        throttle_pwm = max(throttle_pwm, thr_cap_rev_pwm)

        # 2. control: steering [0], throttle [1]
        cmd = "setpwm {0} {1}\n".format(steering_pwm, throttle_pwm)
        ser.write(cmd)
        DEBUG("{:.3f} SETPWM {} {}".format(ts-init_ts, steering_pwm, throttle_pwm))
        
        # 3. record data
        if use_dnn == False and throttle_pwm == thr_cap_ffw_pwm:
            # increase frame_id
            frame_id += 1

            # write input (angle)
            text = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
            keyfile.write(text)
            print "{:.3f},{},{:.3f}".format(ts-init_ts, frame_id, angle)

            # (optional) embedded ts,frame,angle into the image frame
            # text = "{:.3f} {} {:.3f}".format(ts-init_ts, frame_id, angle)
            # cv2.putText(frame, text, (10,50),
            #     cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,255,255))
            
            # write to a video stream
            vidfile.write(frame)

            # (optional) write to a jpg file for each frame
            # cv2.imwrite("out-image-"+str(frame_id)+".jpg", frame)

        # 4. statistics update
        dur = time.time() - ts
        if dur > period:
             print "{:.3f} deadline miss. dur={:.03f}".format(ts-init_ts, dur)
        if ts - init_ts > 1.0:
            # ignore first 1 sec.
            dstat.append(dur)
             
        # 5. read a frame --> THIS MAY BLOCK.
        ret, frame = cap.read() # blocking. period = 1/cfg.cam_fps.
        ts = time.time() 
