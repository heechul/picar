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
import visualize
import time

import local_common as cm
import data_shuffled as data

#Stop warnings related to tensorflow build from showing up
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

epoch_ids = [23] #sorted(list(set(itertools.chain(*params.epochs.values()))))

for epoch_id in epoch_ids:
    print '---------- processing video for epoch {} ----------'.format(epoch_id)
    if not params.givenData:
        if not os.path.exists("datasets/dataset{}/out-mencoder2.avi".format(epoch_id)): #If the video used for visualization doesn't exist
            assert os.path.exists("datasets/dataset{}/out-mencoder.avi".format(epoch_id)) #Make sure a video exists
            os.system("ffmpeg -i datasets/dataset%i/out-mencoder.avi -vf scale=1280:720 datasets/dataset%i/out-mencoder2.avi" % (epoch_id, epoch_id)) #Create a copy video that is resized to be 1280x720 so that it can be visualized
        vid_path = "datasets/dataset{0}/out-mencoder2.avi".format(epoch_id) 
    else:
        vid_path = cm.jn(params.data_dir, 'epoch{:0>2}_front.mkv'.format(epoch_id))
    assert os.path.isfile(vid_path)
    frame_count = cm.frame_count(vid_path)
    cap = cv2.VideoCapture(vid_path)

    machine_steering = []
    timeArr = []

    print 'performing inference...'
    time_start = time.time()
    for frame_id in xrange(frame_count):
        ret, img = cap.read()
        assert ret

        img = preprocess.preprocess(img)

        frameTime = time.time() #Get the start time of the angle calculation
        deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] 
        timeArr.append(time.time() - frameTime) #Add angle calculation time to time array

        machine_steering.append(deg)

    cap.release()

    fps = frame_count / (time.time() - time_start)
    
    print 'completed inference, total frames: {}, average fps: {} Hz'.format(frame_count, round(fps, 1))
    
    print 'performing visualization...'
    startTime = time.time()
    visualize.visualize(epoch_id, machine_steering, params.out_dir, timeArr,
                        verbose=True, frame_count_limit=None)
    print "Visualization took a total of %i seconds" % (time.time() - startTime)
    
    
