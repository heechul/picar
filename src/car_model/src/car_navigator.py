#!/usr/bin/env python
import roslib; roslib.load_manifest('car_model')
import math, os, rospy, signal, sys, tf
from time import sleep
from threading import Thread
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3


class Car_Navigator:

    def __init__(self):
        self.initial_pose = None; self.final_goal = None;
        self.initial_x = None; self.initial_y = None; self.initial_z = None;
        self.goal_x = None; self.goal_y = None; self.goal_z = None;

    def receive_initial(self, msg):
        self.initial_pose = msg
        self.initial_x = msg.pose.pose.position.x
        self.initial_y = msg.pose.pose.position.y
        self.initial_z = msg.pose.pose.position.z



    def receive_goal(self, msg):
        self.final_goal = msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z



    def path_planning(self):
        None


    def path_execution(self):
        None


    def listener(self):
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.receive_initial)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.receive_goal)
        rospy.spin()

def signal_handler(signal, frame):
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node("car_navigator", anonymous=True)
    nav = Car_Navigator()
    signal.signal(signal.SIGINT, signal_handler)

    try:
        nav.listener()
    except rospy.ROSInterruptException:
        pass
