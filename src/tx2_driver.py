#!/usr/bin/env python
import rospy
import math
import serial

import os
import time
import atexit

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
degree2rad = math.pi / 180.0

precision = 2 #round to this number of digits

yaw_offset = 0 #used to align animation upon key press
yaw_measured = 0

ser = serial.Serial('/dev/ttyACM0', 115200)

def turnOff():
	os.system("killall -HUP mencoder")
	
atexit.register(turnOff)

def right(rad):
    ser.write(b'r\n')
    print "right"

def left(rad):
    ser.write(b'l\n')
    print "left"

def center():
    ser.write(b'c\n')
    print "center"

def ffw(vel):
    ser.write(b'f\n')        
    print "forward"

def rew(vel):
    ser.write(b'b\n')        
    print "backward"

def stop():
    ser.write(b's\n')
    print "stop"
    
def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" %
                  (msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" %
                  (msg.angular.x, msg.angular.y, msg.angular.z))
    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities
    # into motor commands

    # speed
    if msg.linear.x < 0.01 and msg.linear.x > -0.01:
        stop()
    elif msg.linear.x > 0.01:
        ffw(msg.linear.x)
    else:
        rew(msg.linear.x)
    
    # turn 
    if msg.angular.z < -0.01:
        right(msg.angular.z)
    elif msg.angular.z > 0.01:
        left(msg.angular.z)
    else:
        center()

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
    
    # print(str(round((yaw)*rad2degrees, precision)) + " / " + str(round((yaw), precision)))
    
    yaw_measured = yaw

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('picar_base', anonymous=True)
    rospy.Subscriber('imu', Imu, processIMU_message)
    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print "***************************************"
    print "       Move base for the PiCar         "
    print "                                       "
    print "Listen '/cmd_vel' topic                "
    print "Control streering and throttle motors  "
    print "***************************************"

            
    listener()
