#!/usr/bin/env python

import roslib; roslib.load_manifest('car_model')
import rospy, tf
from geometry_msgs.msg import Point, Pose, TransformStamped, Twist, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


joint_state = None
odom_trans_joint = None
odom_trans_pose = None
joint_pub = None
broadcaster = None


def joint_publisher(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))





    # update joint_state

    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = [
        "front_axle_to_base" , "back_axle_to_base" ,
        "front_left_wheel_to_front_axle" , "front_right_wheel_to_front_axle" ,
        "back_left_wheel_to_back_axle" , "back_right_wheel_to_back_axle"
    ]


    if msg.linear.y == 0: # move

        if msg.linear.x > 0: # if forward

            if msg.angular.z > 0: # u
                joint_state.position = [ 0, 0, 0.785, 0.393, 3.14*msg.angular.z, 3.14*msg.angular.z ]

            elif msg.angular.x == 0: # i
                joint_state.position = [ 0, 0, 0, 0, 3.14*msg.angular.z, 3.14*msg.angular.z ]

            else: # o
                joint_state.position = [ 0, 0, -0.383, -0.785, -3.14*msg.angular.z, -3.14*msg.angular.z ]

        elif msg.linear.x == 0: # if rotate

            if msg.angular.z > 0: # j
                joint_state.position = [ 0, 0, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z ]

            elif msg.angular.x == 0: # k
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: # l
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z ]

        else: # if reverse

            if msg.angular.z > 0: # m
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z ]

            elif msg.angular.x == 0: # ,
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: # .
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z ]



    # the following section is just dummy code

    else: # strafe

        if msg.linear.x > 0: # if forward

            if msg.angular.z > 0: #
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z]

            elif msg.angular.x == 0: #
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: # o
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z]

        elif msg.linear.x == 0: # if angle

            if msg.angular.z > 0: #
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z]

            elif msg.angular.x == 0: #
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: # o
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z]

        else: # if reverse

            if msg.angular.z > 0: #
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z]

            elif msg.angular.x == 0: #
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: #
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z]

    # end of dummy code section





    # update transform


    odom_trans_joint.header.stamp = rospy.Time.now()
    odom_trans_joint.header.frame_id = "odom"
    odom_trans_joint.child_frame_id = "base_link"

    odom_trans_joint.transform.translation.x = msg.linear.x
    odom_trans_joint.transform.translation.y = msg.linear.y
    odom_trans_joint.transform.translation.z = abs(msg.angular.z)

    odom_trans_joint.transform.rotation = tf.transformations.quaternion_from_euler(0, -1.57, 0)

    # send the joint state and transform
    joint_pub.publish(joint_state)
    broadcaster.sendTransform(
        (msg.linear.x, msg.linear.y, msg.angular.z),
        odom_trans_joint.transform.rotation,
        rospy.Time.now(),
        odom_trans_joint.child_frame_id,
        odom_trans_joint.header.frame_id
    )








def robot_movement(msg):

    rospy.spin()







def listener():
    rospy.init_node("joint_state_publisher")
    rospy.Subscriber("/cmd_vel", Twist, joint_publisher)
    rospy.spin()

if __name__ == '__main__':
    joint_state = JointState()
    joint_state.header = Header()

    odom_trans_joint = TransformStamped()
    odom_trans_joint.header = Header()

    broadcaster = tf.TransformBroadcaster()
    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
