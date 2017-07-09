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
import visualize_parallel
# import visualize
import time
import math

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
# epoch_ids = [6]

for epoch_id in epoch_ids:
    print '---------- processing video for epoch {} ----------'.format(epoch_id)
    # vid_path = cm.jn(params.data_dir, 'epoch{:0>2}_front.mkv'.format(epoch_id))

    vid_scaled_path = cm.jn(params.data_dir, 'out-video-{}-scaled.avi'.format(epoch_id))
    if not os.path.exists(vid_scaled_path):
        vid_path = cm.jn(params.data_dir, 'out-video-{}.avi'.format(epoch_id))
        assert os.path.isfile(vid_path)
        os.system("ffmpeg -i " + vid_path + " -vf scale=1280:720 " + vid_scaled_path)
    vid_path = vid_scaled_path
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

        machine_steering.append(deg)

    cap.release()

    fps = frame_count / (time.time() - time_start)
    
    print 'completed inference, total frames: {}, average fps: {} Hz'.format(frame_count, round(fps, 1))

    print "Machine Steering:", machine_steering

    print 'performing visualization...'
    visualize_parallel.visualize(epoch_id, machine_steering, params.out_dir,
                        verbose=True, frame_count_limit=None)
    
    
