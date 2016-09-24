#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
precision = 2 #round to this number of digits

yaw_offset = 0 #used to align animation upon key press
yaw_measured = 0

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import os
import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOff():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
	os.system("killall -HUP mencoder")
	
atexit.register(turnOff)

steeringMotor = mh.getMotor(1)

MAX_STEERING = 50 # safe margin.
MAX_SPEED = 1500  # pwm  1500us/20ms = max full throttle
NEU_SPEED = 1270  # pwm  1270us/20ms = stop
MIN_SPEED = 950   # pwm   950us/20ms = reverse full throttle

currentSpeed = NEU_SPEED


def right(rad):
    speed = int(-rad / 0.5 * 100)
    steeringMotor.run(Adafruit_MotorHAT.FORWARD)
    steeringMotor.setSpeed(speed)
    print "right"

def left(rad):
    speed = int(rad / 0.5 * 100)
    steeringMotor.run(Adafruit_MotorHAT.BACKWARD)
    steeringMotor.setSpeed(speed)
    print "left"

def center():
    steeringMotor.run(Adafruit_MotorHAT.RELEASE);
    print "center"

def ffw(vel):
    speed = int(vel / 0.5 * 150) + NEU_SPEED
    os.system("echo 0=%dus > /dev/servoblaster" % speed)
    print "accel:", speed
    
def stop():
    print "stop"
    os.system("echo 0=%dus > /dev/servoblaster" % NEU_SPEED)
    
    
def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))
    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands

    # speed
    if msg.linear.x < 0.01 and msg.linear.x > -0.01:
        stop()
    else:
        ffw(msg.linear.x)
    
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
    
    print(str(round((yaw)*rad2degrees, precision)) + " / " + str(round((yaw), precision)))
    
    yaw_measured = yaw

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('picar_base', anonymous=True)
    sub = rospy.Subscriber('imu', Imu, processIMU_message)
    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
