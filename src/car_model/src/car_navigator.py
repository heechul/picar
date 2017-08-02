#!/usr/bin/env python
import roslib; roslib.load_manifest('car_model')
import math, os, rospy, signal, sys, tf
from time import sleep
from threading import Thread
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist, Vector3


class Car_Navigator:

    def __init__(self):
        self.twist_pub = rospy.Publisher("/Self_Driver/cmd_vel", Twist, queue_size=50)
        self.initial_pose = None; self.final_goal = None;
        self.received_initial = False; self.received_goal = False;
        self.initial_x = 0; self.initial_y = 0; self.initial_z = 0;
        self.goal_x = 0; self.goal_y = 0; self.goal_z = 0;
        self.current_x = 0; self.current_y = 0; self.current_z = 0

    def receive_initial(self, msg):
        self.initial_pose = msg; self.received_initial = True;
        self.initial_x = msg.pose.pose.position.x
        self.initial_y = msg.pose.pose.position.y
        self.initial_z = msg.pose.pose.position.z

        if self.received_goal:
            self.path_execution()


    def receive_goal(self, msg):
        self.final_goal = msg; self.received_goal = True;
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z

        if self.received_initial:
            self.path_execution()


    def determine_current_location(self):
        None


    def path_planning(self):
        None


    def path_execution(self):
        twist = Twist()
        twist.linear.y = 0; twist.linear.z = 0; twist.angular.x = 0; twist.angular.y = 0;
        twist.linear.x = 0
        twist.angular.z = 0

        # arctan(y/x)
        self.twist_pub.publish(twist)




    def listener(self):
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.receive_initial)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.receive_goal)
        rospy.Subscriber("tf", tf)
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
