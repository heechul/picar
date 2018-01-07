#!/usr/bin/env python 
from __future__ import division

import model
import cv2
import subprocess as sp
import itertools
import params
import sys
import os
import preprocess
# import visualize
import time
import math
import csv
import numpy as np
import resource
import tensorflow as tf

import local_common as cm

def deg2rad(deg):
        return deg * math.pi / 180.0
def rad2deg(rad):
        return 180.0 * rad / math.pi
        
numCore = 1
sess = tf.InteractiveSession(config=tf.ConfigProto(intra_op_parallelism_threads=numCore, inter_op_parallelism_threads=numCore, allow_soft_placement=True))
saver = tf.train.Saver()
model_name = params.model_name
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

epoch_ids = sorted(list(set(itertools.chain(*params.epochs.values()))))

# epoch_ids = [6] # DBG - heechul

cap_time_list = []
process_time_list = []
angle_time_list = []
frame_time_list = []

frameNum = 0

cap = cv2.VideoCapture(0)

for epoch_id in epoch_ids:
    print '---------- processing video for epoch {} ----------'.format(epoch_id)
    # vid_path = cm.jn(params.data_dir, 'epoch{:0>2}_front.mkv'.format(epoch_id))

    vid_path = cm.jn(params.data_dir, 'out-video-{}.avi'.format(epoch_id))
    assert os.path.isfile(vid_path)
    frame_count = cm.frame_count(vid_path)
    #cap = cv2.VideoCapture(vid_path)

    machine_steering = []

    print 'performing inference...'
    time_start = time.time()
    for frame_id in xrange(frame_count):
        if frameNum < 1001:
            cap_start = time.time()
            ret, img = cap.read()
            assert ret
            
            prep_start = time.time()

            img = preprocess.preprocess(img)

            pred_start = time.time()
            #with sess.as_default():
            rad = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0]
            deg = rad2deg(rad)
            #with sess2.as_default():
                #rad2 = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0]
                #deg2 = rad2deg(rad2)
            pred_end   = time.time()

            cap_time = cap_start - prep_start #Time to capture the frame
            prep_time = pred_start - prep_start #Time to preprocess the frame
            pred_time = pred_end - pred_start #Time to predict the angle
            total_time = pred_end - cap_start #Total time to process the frame
            
            cap_time_list.append(prep_time * 1000)
            process_time_list.append(prec_time * 1000)
            angle_time_list.append(pred_time * 1000)
            frame_time_list.append(total_time * 1000)

            print 'pred: {} deg. took {} ms'.format(deg, total_time * 1000)
            # print 'pred: {} deg (rad={})'.format(deg, rad)
            machine_steering.append(deg)
            
            frameNum+=1

    #cap.release()

    fps = frame_count / (time.time() - time_start)
    
    print 'completed inference, total frames: {}, average fps: {} Hz'.format(frame_count, round(fps, 1))

    print "Machine Steering:", machine_steering

    # print 'performing visualization...'
    # visualize.visualize(epoch_id, machine_steering, params.out_dir,
    #                     verbose=True, frame_count_limit=None)
    
cap.release()

frame_time_list.pop(0)
process_time_list.pop(0)
angle_time_list.pop(0)
cap_time_list.pop(0)
    
numMisses = 0
numNinety = 0
numEighty = 0
numSeventy = 0
numSixty = 0
numFifty = 0
numLower = 0
for i in xrange(len(frame_time_list)):
    if frame_time_list[i] > 100:
        numMisses+=1
    elif frame_time_list[i] > 90:
        numNinety+=1
    elif frame_time_list[i] > 80:
        numEighty+=1
    elif frame_time_list[i] > 70:
        numSeventy+=1
    elif frame_time_list[i] > 60:
        numSixty+=1
    elif frame_time_list[i] > 50:
        numFifty+=1
    else:
        numLower+=1
        
print "count:", len(frame_time_list)
print "mean:", np.mean(frame_time_list)
print "max:", np.max(frame_time_list)
print "99.999pct:", np.percentile(frame_time_list, 99.999)
print "99.99pct:", np.percentile(frame_time_list, 99.99)
print "99.9pct:", np.percentile(frame_time_list, 99.9)
print "99pct:", np.percentile(frame_time_list, 99)
print "min:", np.min(frame_time_list)
print "median:", np.median(frame_time_list)
print "stdev:", np.std(frame_time_list)
print "misses: ", numMisses
print ">90 ms: ", numNinety
print ">80 ms: ", numEighty
print ">70 ms: ", numSeventy
print ">60 ms: ", numSixty
print ">50 ms: ", numFifty
print "<= 50 ms: ", numLower 

firstLine = True

#Open the csv file
with open("Timing/time_data.csv".format(numCore), 'a') as file:
    writer = csv.writer(file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL) #Create a writer
    writer.writerow([numCore, len(frame_time_list), np.mean(frame_time_list), np.max(frame_time_list), np.percentile(frame_time_list, 99.999), np.percentile(frame_time_list, 99.99), np.percentile(frame_time_list, 99.9),
            np.percentile(frame_time_list, 99), np.min(frame_time_list), np.median(frame_time_list), np.std(frame_time_list), numMisses, numNinety, numEighty, numSeventy, numSixty, numFifty, numLower]) #Write all of the data points to the csv file
            
#Open the csv file
with open("Timing/time_{}c.csv".format(numCore), 'a') as file:
    writer = csv.writer(file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL) #Create a writer
    writer.writerow(cap_time_list) #Write the time array to the csv file
    writer.writerow(process_time_list)
    writer.writerow(angle_time_list)
    writer.writerow(frame_time_list)
    writer.writerow('')

 
