#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from visual import *
from geometry_msgs.msg import Twist

import math
import wx

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
precision = 2 #round to this number of digits

yaw_offset = 0 #used to align animation upon key press
yaw_measured = 0
yaw_setpoint = 0

#Create shutdown hook to kill visual displays
def shutdown_hook():
    print "Killing displays"
    wx.Exit()

#register shutdown hook
rospy.on_shutdown(shutdown_hook)

def processIMU_message(imuMsg):
    global yaw_offset, yaw_measured

    roll=0
    pitch=0
    yaw=0

    quaternion = (
      imuMsg.orientation.x,
      imuMsg.orientation.y,
      imuMsg.orientation.z,
      imuMsg.orientation.w)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)

    #add align offset to yaw
    yaw += yaw_offset

    axis=(-cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
    up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw),cos(roll)*cos(pitch))
    
    #remove yaw_offset from yaw display
    # print(str(round((yaw-yaw_offset)*rad2degrees, precision)) + " / " + str(round((yaw-yaw_offset), precision)))

    # measurement data update
    yaw_measured = (yaw - yaw_offset)



if __name__=="__main__":
    # imu is published at 50Hz
    sub = rospy.Subscriber('imu', Imu, processIMU_message)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node("yaw controller")

    r = rospy.Rate(20) # 20Hz. 50ms/iter

    speed = .65
    turn = 1
    ed = 0
    
    # PID parameters
    Kp = 0.5; Kd = 0.1
    
    # main loop
    while not rospy.is_shutdown():
        e = yaw_measured - yaw_setpoint
        print "Err=", e, "Yaw(measure):", yaw_measured
        g = Kp * e + Kd * (e - ed)
        # print "gain:", g
    
        twist = Twist()

        # directional vector
        twist.linear.x = speed;
        twist.linear.y = twist.linear.z = 0
        
        # angular vector
        twist.angular.z = g * turn    
        twist.angular.x = 0; twist.angular.y = 0;

        # correct error of the previous run
        ed = e

        print "Twist:", twist
        # send control message
        pub.publish(twist)

        # sleep
        r.sleep()
    
    
