#!/usr/bin/env python 
from __future__ import division

import tensorflow as tf
import model_btn as model
import cv2
import subprocess as sp
import itertools
import params_btn as params
import sys
import os
import preprocess
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
model_load_path = cm.jn(params.save_dir, params.model_load_file)
saver.restore(sess, model_load_path)

epoch_ids = sorted(list(set(itertools.chain(*params.epochs.values()))))

# epoch_ids = [6] # DBG - heechul

for epoch_id in epoch_ids:
    print '---------- processing video for epoch {} ----------'.format(epoch_id)
    # vid_path = cm.jn(params.data_dir, 'epoch{:0>2}_front.mkv'.format(epoch_id))

    vid_path = cm.jn(params.data_dir, 'out-video-{}.avi'.format(epoch_id))
    assert os.path.isfile(vid_path)
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
        btn = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})
        pred_end   = time.time()

        prep_time = pred_start - prep_start
        pred_time = pred_end - pred_start

        print 'pred: {} took {} ms'.format(btn, pred_time * 1000)

        machine_steering.append(btn)

    cap.release()

    fps = frame_count / (time.time() - time_start)
    
    print 'completed inference, total frames: {}, average fps: {} Hz'.format(frame_count, round(fps, 1))

    print "Machine Steering:", machine_steering

    # print 'performing visualization...'
    # visualize.visualize(epoch_id, machine_steering, params.out_dir,
    #                     verbose=True, frame_count_limit=None)
