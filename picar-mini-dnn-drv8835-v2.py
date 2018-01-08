#!/usr/bin/python
import os
import time
import atexit
import serial
import cv2
import math
import numpy as np
import sys
from threading import Thread

print ("Load TF")
import tensorflow as tf
import params
model = __import__(params.model)
import local_common as cm
import preprocess

print ("Load Model")
sess = tf.InteractiveSession()    
saver = tf.train.Saver()
model_load_path = cm.jn(params.save_dir, params.model_load_file)
saver.restore(sess, model_load_path)
print ("Done..")
    
from pololu_drv8835_rpi import motors, MAX_SPEED

def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

def stop():
    global cur_speed
    cur_speed = 0
    motors.motor2.setSpeed(int(cur_speed))
        
def ffw():
    global cur_speed
    # cur_speed = min(MAX_SPEED, cur_speed + MAX_SPEED/10)
    cur_speed = SET_SPEED    
    motors.motor2.setSpeed(int(cur_speed))

def rew():
    global cur_speed        
    # cur_speed = max(-MAX_SPEED, cur_speed - MAX_SPEED/10)
    cur_speed = SET_SPEED
    motors.motor2.setSpeed(-int(cur_speed))

def center():
    motors.motor1.setSpeed(0)
def left():
    motors.motor1.setSpeed(MAX_SPEED)
def right():
    motors.motor1.setSpeed(-MAX_SPEED)


def get_control(degree):
    if degree < 15 and degree > -15:
        return 0
    elif degree >= 15:
        return 1
    elif degree <= -15:
        return -1

def get_ctl_str(control):
    if control == -1:
        return "left"
    elif control == 0:
        return "center"
    elif control == 1:
        return "right"
    else:
        raise ValueError
    
def turnOff(stat):
    stop()
    center()
    print "Control loop statistics"
    print "======================="
    print "min/avg/99pct/max (sec): {:.3f} {:.3f} {:.3f} {:.3f}".format(
        np.min(stat), np.mean(stat), np.percentile(stat, 99), np.max(stat))
    dmiss_cnt = sum(i > period for i in stat)
    print "deadline miss(es): {}/{} {:.1f}%".format(dmiss_cnt, len(stat),
                                float(dmiss_cnt)/len(stat)*100)

def g_tick():
    t = time.time()
    count = 0
    while True:
        count += 1
        yield max(t + count*period - time.time(),0)

class Camera:
    def __init__(self, res=(320, 240), fps=30):
        print "Initilize camera."
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, res[0]) # width
        self.cap.set(4, res[1]) # height
        self.cap.set(5, fps)
        self.frame = None
        self.enabled = True
        
    def update(self):
        while self.enabled:
            ret, self.frame = self.cap.read() # blocking read.

    def read_frame(self):
        return self.frame

    def shutdown(self):
        print ("Close the camera.")
        self.enabled = False;        
        self.cap.release()

cfg_width = 320
cfg_height = 240
cfg_fps = 30

cam = Camera((cfg_width, cfg_height), cfg_fps)
cam_thr = Thread(target=cam.update, args=())
cam_thr.start()
time.sleep(2)

frame_id = 0
angle = 0.0
btn   = 107
period = 0.05 # sec (=50ms)

motors.setSpeeds(0, 0)
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
fourcc = cv2.cv.CV_FOURCC(*'XVID')
vidfile = cv2.VideoWriter('out-video.avi', fourcc, 
	cfg_fps, (cfg_width,cfg_height))
keyfile = open('out-key.csv', 'w+')
keyfile_btn = open('out-key-btn.csv', 'w+')
keyfile.write("ts_micro,frame,wheel\n")
keyfile_btn.write("ts_micro,frame,btn,speed\n")
rec_start_time = 0
SET_SPEED = MAX_SPEED * 6 / 10 
cur_speed = SET_SPEED
print "MAX speed:", MAX_SPEED
print "cur speed:", cur_speed

dstat = []
atexit.register(turnOff, stat=dstat)

null_frame = np.zeros((cfg_width,cfg_height,3), np.uint8)
cv2.imshow('frame', null_frame)
ch = cv2.waitKey(1) & 0xFF

if len(sys.argv) == 2:
    SET_SPEED = MAX_SPEED * int(sys.argv[1]) / 10 
    print "Set new speed: ", SET_SPEED

g = g_tick()

print ("Enter the main loop.")
m_ctrl = -99
view_video = False
ts = init_ts = time.time()
while True:
    time.sleep(next(g))    
    ts = time.time()

    # 0. read a frame
    frame = cam.read_frame()
        
    if view_video == True:
        cv2.imshow('frame', frame)

    # 1. machine input
    img = preprocess.preprocess(frame)
    # print ("Perform an inference operation.")
    angle_dnn = model.y.eval(feed_dict={model.x: [img],
                                        model.keep_prob: 1.0})[0][0]
    angle_dnn = rad2deg(angle_dnn)
    m_ctrl = get_control(angle_dnn)

    # 2. human input (safety backup)
    h_ctrl = -99    
    ch = cv2.waitKey(1) & 0xFF

    # 2.1. steering key    
    if ch == ord('j'):
        h_ctrl = -1
        angle = deg2rad(-30)
        btn = ch
    elif ch == ord('k'):
        h_ctrl = 0
        angle = deg2rad(0)
        btn = ch
    elif ch == ord('l'):
        h_ctrl = 1
        angle = deg2rad(30)
        btn = ch
    # 2.1. service task        
    elif ch == ord('a'):
        ffw()
        print "accel"
    elif ch == ord('s'):
        stop()
        print "stop"                
    elif ch == ord('z'):
        rew()
        print "reverse"
    elif ch == ord('q'):
        break
    elif ch == ord('t'):
        print "toggle video mode"
        if view_video == False:
            view_video = True
        else:
            view_video = False
	    
    # 3. decision
    if m_ctrl == h_ctrl or h_ctrl == -99:
        # machine == human or there's no human input 
        ctrl = m_ctrl
        ctrl_by = "M"
    else:
        # if there's human input, use it.
        ctrl = h_ctrl
        ctrl_by = "H"
        
    # 4. control
    if ctrl == 0:
        center()
    elif ctrl == 1:
        right()
    elif ctrl == -1:
        left()

    # statistics update
    dur = time.time() - ts
    print ("%s %.3f %d %s %d (ms)" %
        (ctrl_by, ts - init_ts, angle_dnn, get_ctl_str(ctrl), int(dur*1000)))
    if dur > period:
        print ("--> deadline miss.")
    if ts - init_ts > 1.0:
        # ignore first 1 sec.
        dstat.append(dur)
        
    if ctrl == h_ctrl:
        # human override.
        # increase frame_id
        frame_id += 1

        # write input (angle)
        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
        keyfile.write(str)
        
        # write input (button: left, center, stop, speed)
        str = "{},{},{},{}\n".format(int(ts*1000), frame_id, btn, cur_speed)
        keyfile_btn.write(str)
        
        # write video stream
        vidfile.write(frame)

        if frame_id >= 1000:
            print "recorded 1000 frames"
            break

stop()
cam.shutdown()
keyfile.close()
keyfile_btn.close()
vidfile.release()
