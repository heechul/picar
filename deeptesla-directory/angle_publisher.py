#!/usr/bin/env python 
from __future__ import division

import tensorflow as tf
import model
import params
import csv
import cv2
import local_common as cm
import os
import preprocess

import rospy
from geometry_msgs.msg import Twist

#Stop source warnings from appearing
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

epoch_id = 23

#Create a node that publishes to cmd_vel
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('rostest')

#Open the model
sess = tf.InteractiveSession()
saver = tf.train.Saver()
model_name = 'model.ckpt'
model_path = cm.jn(params.save_dir, model_name)
saver.restore(sess, model_path)

#Open the video and get the number of frames
vid_path = "datasets/dataset{0}/out-mencoder.avi".format(epoch_id)
assert os.path.isfile(vid_path)
frame_count = cm.frame_count(vid_path)
cap = cv2.VideoCapture(vid_path)

#Get the predicted angle from the model for each frame and publish the angle
for frame_id in xrange(frame_count):
    ret, img = cap.read()
    assert ret
    img = preprocess.preprocess(img)
    deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] 
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = deg
    pub.publish(twist)
