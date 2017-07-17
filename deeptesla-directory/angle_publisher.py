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
import time

import rospy
from geometry_msgs.msg import Twist

TARGET_SPEED = 0

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


'''
cap = cv2.VideoCapture(0)

twist = Twist()
twist.linear.x = 1; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0

while(True):
    startTime = time.time()
    ret, frame = cap.read()
    frame = preprocess.preprocess(frame)
    #deg = model.y.eval(feed_dict={model.x: [frame], model.keep_prob: 1.0})[0][0]
    #twist.angular.z = deg	
    #pub.publish(twist)
    endTime = (time.time() - startTime) * 1000
    print endTime
'''

#Open the video and get the number of frames
vid_path = "datasets/dataset{0}/out-mencoder.avi".format(epoch_id)
assert os.path.isfile(vid_path)
frame_count = cm.frame_count(vid_path)
cap = cv2.VideoCapture(vid_path)

tempAngle = 0 #Hold the angle

totalTime = time.time() #Used to get the total time for publishing all angles

#Get the predicted angle from the model for each frame and publish the angle
for frame_id in xrange(frame_count):
    ret, img = cap.read() #Get the frame
    assert ret #Make sure the frame exists
    img = preprocess.preprocess(img) #Process the image
    deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
    deg = round(deg * 8) / 8 #Round the angle to the nearest eighth
    print deg #Print the rounded angle
	
	#Create a twist message that determines if a turn is necessary and publish it
    twist = Twist()
    twist.linear.x = TARGET_SPEED; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = (tempAngle - deg) * 2
    pub.publish(twist)
	
	#Update the temporary angle value to the angle of the current frame
    tempAngle = deg

#Create and publish a final twist message that stops the car if necessary
twist = Twist()
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

#Print the total time it took to publish all of the predicted angles
print round((time.time() - totalTime))
