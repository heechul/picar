#!/usr/bin/env python
import roslib; roslib.load_manifest('car_model')
import math, rospy, tf
from time import sleep
from threading import Thread
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState



class Twist_Driver:

    def __init__(self):
        self.joint_state = JointState()
        self.odom = Odometry()
        self.trans = TransformStamped()
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=50)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.broadcaster = tf.TransformBroadcaster()
        self.thr = Thread(target = self.publish_messages)


        self.joint_state.header = Header()
        self.joint_state.header.frame_id = "base_footprint"
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = [
            "base_footprint_joint" , "camera_joint" ,
            "front_axle_to_base" , "back_axle_to_base" ,
            "front_left_wheel_to_front_axle" , "front_right_wheel_to_front_axle" ,
            "back_left_wheel_to_back_axle" , "back_right_wheel_to_back_axle"
        ]
        self.joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0]

        quat_1 = tf.transformations.quaternion_from_euler(0,0,0)
        quat_2 = tf.transformations.quaternion_from_euler(0,0,(math.pi/2))

        self.trans.header = Header()
        self.trans.header.frame_id = "odom"
        self.trans.child_frame_id = "base_footprint"
        self.trans.header.stamp = rospy.Time.now()
        self.trans.transform.translation.x = 0
        self.trans.transform.translation.y = 0
        self.trans.transform.translation.z = 0
        self.trans.transform.rotation = quat_1

        self.odom.header = Header()
        self.odom.header.frame_id = "odom";
        self.odom.child_frame_id = "base_footprint"
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = 0
        self.odom.pose.pose.position.y = 0
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = quat_2[0]
        self.odom.pose.pose.orientation.y = quat_2[1]
        self.odom.pose.pose.orientation.z = quat_2[2]
        self.odom.pose.pose.orientation.w = quat_2[3]
        self.odom.twist.twist = Twist(Vector3(0,0,0), Vector3(0,0,0))

        self.f_left = 0; self.f_right = 0; self.back_wheels = 0;
        self.original_speed = 0.5; self.current_linx = 0.5; self.turn = 1;
        self.angle = (math.pi/2); self.previous_angle = (math.pi/2);
        self.comparison_angle = self.angle
        self.first_message = False; self.turn_left = False; self.turn_right = False;
        self.move_x = 0; self.move_y = 0; self.move_z = 0;
        self.vx = 0.1; self.vy = 0.1; self.vth = self.angle;
        self.current_time = rospy.Time.now(); self.last_time = rospy.Time.now();




    def receive_twist(self, msg):
        self.first_message = True; self.previous_angle = self.angle
        self.last_time = self.current_time; self.current_time = rospy.Time.now()

            # increment speed variables
        if not abs(msg.linear.x) == abs(self.current_linx):
            self.current_linx = msg.linear.x;
            #self.vx = msg.linear.x; self.vy = self.vx;

            # else, only a change in angular speed has been detected
        if not abs(msg.angular.z) == abs(self.vth):
            self.vth = msg.angular.z
            self.turn = msg.angular.z / self.original_speed


        if msg.linear.y == 0: # move

            if msg.linear.x > 0: # if forward

                if msg.angular.z > 0: # u
                    self.f_left = 0.785; self.f_right = 0.393; self.back_wheels += 3.14;
                    self.angle += (math.pi/72)

                elif msg.angular.z == 0: # i
                    self.f_left = 0; self.f_right = 0; self.back_wheels += 3.14;

                else: # o
                    self.f_left = -0.393; self.f_right = -0.785; self.back_wheels += 3.14;
                    self.angle -= (math.pi/72)


            elif msg.linear.x == 0: # if rotate

                if msg.angular.z > 0: # j
                    self.f_left = 0.785; self.f_right = 0.393; self.angle += (math.pi/72);

                elif msg.angular.z == 0: # k
                    self.f_left = 0; self.f_right = 0;

                else: # l
                    self.f_left = -0.393; self.f_right = -0.785; self.angle -= (math.pi/72);


            else: # if reverse

                if msg.angular.z < 0: # m
                    self.f_left = 0.785; self.f_right = 0.393; self.back_wheels -= 3.14;
                    self.angle -= (math.pi/72)

                elif msg.angular.z == 0: # ,
                    self.f_left = 0; self.f_right = 0; self.back_wheels -= 3.14;

                else: # .
                    self.f_left = -0.393; self.f_right = -0.785; self.back_wheels -= 3.14;
                    self.angle += (math.pi/72)


        self.comparison_angle = self.angle
        while self.comparison_angle < 0:
            self.comparison_angle += 2*math.pi
        while self.comparison_angle > 2*math.pi:
            self.comparison_angle -= 2*math.pi


        if (self.comparison_angle == 0 or self.comparison_angle == (math.pi/2) or self.comparison_angle == (math.pi*1.5)):
            self.turn_left = False; self.turn_right = False;
        else:
            self.turn_left = (self.previous_angle < self.angle)
            self.turn_right = (self.previous_angle > self.angle)

        self.move_x = msg.linear.x * self.vx * math.cos(self.angle)
        self.move_y = msg.linear.x * self.vy * math.sin(self.angle)
        self.move_z = msg.linear.z

        self.joint_state.position = [0, 0, 0, 0, self.f_left, self.f_right, self.back_wheels, self.back_wheels]




    def publish_messages(self):

        while not rospy.is_shutdown():

            if self.first_message and not (self.f_left == 0 or self.f_right == 0):

                self.turn_left = self.previous_angle < self.angle
                self.turn_right = self.previous_angle > self.angle

                self.previous_angle = self.angle
                if self.turn_left:
                    self.angle += (math.pi/72)
                elif self.turn_right:
                    self.angle -= (math.pi/72)

                self.comparison_angle = self.angle

                self.move_x = self.move_x * math.cos(self.angle) / math.cos(self.previous_angle)
                self.move_y = self.move_y * math.sin(self.angle) / math.sin(self.previous_angle)

            quat_1 = tf.transformations.quaternion_from_euler(0,0, (self.angle - math.pi/2) )
            quat_2 = tf.transformations.quaternion_from_euler(0,0,self.angle)

            self.last_time = self.current_time; self.current_time = rospy.Time.now()

            self.joint_state.header = Header()
            self.joint_state.header.frame_id = "base_footprint"
            self.joint_state.header.stamp = self.current_time
            self.joint_pub.publish(self.joint_state)

            self.trans.header = Header()
            self.trans.header.frame_id = "odom"
            self.trans.child_frame_id = "base_footprint"
            self.trans.header.stamp = self.current_time;
            self.trans.transform.translation.x += self.move_x
            self.trans.transform.translation.y += self.move_y
            self.trans.transform.translation.z += self.move_z
            self.trans.transform.rotation = quat_1
            self.broadcaster.sendTransform(
                (self.trans.transform.translation.x, self.trans.transform.translation.y, self.trans.transform.translation.z),
                self.trans.transform.rotation,
                self.current_time,
                self.trans.child_frame_id,
                self.trans.header.frame_id
            )

            self.odom.header = Header()
            self.odom.header.frame_id = "odom";
            self.odom.child_frame_id = "base_footprint"
            self.odom.header.stamp = self.current_time
            self.odom.pose.pose.position.x += self.move_x
            self.odom.pose.pose.position.y += self.move_y
            self.odom.pose.pose.position.z += self.move_z
            self.odom.pose.pose.orientation.x = quat_2[0]
            self.odom.pose.pose.orientation.y = quat_2[1]
            self.odom.pose.pose.orientation.z = quat_2[2]
            self.odom.pose.pose.orientation.w = quat_2[3]
            self.odom_pub.publish(self.odom);

            sleep(0.1)




    def listener(self):
        rospy.Subscriber("cmd_vel", Twist, self.receive_twist)
        self.thr.start()
        rospy.spin()




if __name__ == '__main__':
    rospy.init_node("twist_driver", anonymous=True)
    driver = Twist_Driver()

    try:
        driver.listener()
    except rospy.ROSInterruptException:
        pass
