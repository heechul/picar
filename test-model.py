#!/usr/bin/env python 
from __future__ import division

import tensorflow as tf
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
import numpy as np
import local_common as cm

def deg2rad(deg):
        return deg * math.pi / 180.0
def rad2deg(rad):
        return 180.0 * rad / math.pi

sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

epoch_ids = sorted(list(set(itertools.chain(*params.epochs.values()))))

# epoch_ids = [6] # DBG - heechul

pred_time_list = []

for epoch_id in epoch_ids:
    print '---------- processing video for epoch {} ----------'.format(epoch_id)
    # vid_path = cm.jn(params.data_dir, 'epoch{:0>2}_front.mkv'.format(epoch_id))

    vid_path = cm.jn(params.data_dir, 'out-video-{}.avi'.format(epoch_id))
    if not os.path.isfile(vid_path):
        continue
    
    frame_count = cm.frame_count(vid_path)
    cap = cv2.VideoCapture(vid_path)

    machine_steering = []

    print 'performing inference...'
    time_start = time.time()
    for frame_id in xrange(frame_count):
        ret, img = cap.read()
        assert ret

        prep_start = time.time()
        img = preprocess.preprocess(img)

        pred_start = time.time()
        rad = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0]
        deg = rad2deg(rad)
        pred_end   = time.time()

        prep_time = pred_start - prep_start
        pred_time = pred_end - pred_start

        # print 'pred: {} deg. took {} ms'.format(deg, pred_time * 1000)
        # print 'pred: {} deg (rad={})'.format(deg, rad)

        pred_time_list.append(pred_time*1000)
        machine_steering.append(deg)

    cap.release()

    fps = frame_count / (time.time() - time_start)
    
    print 'completed inference, total frames: {}, average fps: {} Hz'.format(frame_count, round(fps, 1))
    # print "Machine Steering:", machine_steering

    # print 'performing visualization...'
    # visualize.visualize(epoch_id, machine_steering, params.out_dir,
    #                     verbose=True, frame_count_limit=None)


print "count:", len(pred_time_list)
print "mean:", np.mean(pred_time_list)
print "max:", np.max(pred_time_list)
print "99.999pct:", np.percentile(pred_time_list, 99.999)
print "99.99pct:", np.percentile(pred_time_list, 99.99)
print "99.9pct:", np.percentile(pred_time_list, 99.9)
print "99pct:", np.percentile(pred_time_list, 99)
print "min:", np.min(pred_time_list)
print "median:", np.median(pred_time_list)
print "stdev:", np.std(pred_time_list)

    
