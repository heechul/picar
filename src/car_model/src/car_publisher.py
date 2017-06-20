#!/usr/bin/env python

import roslib; roslib.load_manifest('car_model')
import rospy, tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState


joint_state = None
transf = None
pose_st = None
joint_pub = None
broadcaster = None

original_linx = 0.5
previous_linx = 0.5
current_linx = 0.5
turn = 1
angle = 1


f_left = 0
f_right = 0
back_wheels = 0

move_x = 0
move_y = 0
move_z = 0



def receive_twist(msg):
    global original_linx, previous_linx, current_linx, turn, move_x, move_y, \
    move_z, angle, f_left, f_right, back_wheels

    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))



    # a change in the linear speed has been detected
    if not abs(msg.linear.x) == abs(current_linx):
        previous_linx = current_linx
        current_linx = msg.linear.x

    elif not abs(msg.angular.z) == abs(turn): # else, only a change in angular speed has been detected
        turn = msg.angular.z / original_linx
        angle = msg.angular.z



    if msg.linear.y == 0: # move

        if msg.linear.x > 0: # if forward

            if msg.angular.z > 0: # u
                f_left = 0.785
                f_right = 0.393
                back_wheels += 3.14*msg.angular.z

            elif msg.angular.z == 0: # i
                f_left = 0
                f_right = 0
                back_wheels += 3.14*msg.angular.z

            else: # o
                f_left = -0.393
                f_right = -0.785
                back_wheels += 3.14*msg.angular.z

        elif msg.linear.x == 0: # if rotate

            if msg.angular.z > 0: # j
                f_left = 0.785
                f_right = 0.393

            elif msg.angular.z == 0: # k
                f_left = 0
                f_right = 0

            else: # l
                f_left = -0.393
                f_right = -0.785

        else: # if reverse

            if msg.angular.z < 0: # m
                f_left = 0.785
                f_right = 0.393
                back_wheels -= 3.14*msg.angular.z

            elif msg.angular.z == 0: # ,
                f_left = 0
                f_right = 0
                back_wheels -= 3.14*msg.angular.z

            else: # .
                f_left = -0.393
                f_right = -0.785
                back_wheels -= 3.14*msg.angular.z





    # strafe code needs to be added

    else: # strafe
        return


    move_x = msg.linear.x
    move_y = msg.linear.y
    move_z = msg.linear.z




    move_joints()
    move_robot()



def move_joints():
    # update joint_state


    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = [
        "front_axle_to_base" , "back_axle_to_base" ,
        "front_left_wheel_to_front_axle" , "front_right_wheel_to_front_axle" ,
        "back_left_wheel_to_back_axle" , "back_right_wheel_to_back_axle"
    ]
    joint_state.position = [ 0, 0, f_left, f_right, back_wheels, back_wheels ]

    # send the joint state and transform
    joint_pub.publish(joint_state)



def move_robot():


    # update transform and pose
    quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)



    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = "odom"
    trans.child_frame_id = "base_link"
    trans.transform.translation.x += move_x*quaternion[2]
    trans.transform.translation.y += move_y*quaternion[3]
    trans.transform.translation.z += move_z
    trans.transform.rotation = quaternion



    broadcaster.sendTransform(
        (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z),
        trans.transform.rotation,
        rospy.Time.now(),
        trans.header.frame_id,
        trans.child_frame_id
    )


    rospy.loginfo("Angle: %f"%(angle))
    rospy.loginfo("Quaternion: [%f, %f, %f, %f]"%(quaternion[0], quaternion[1], quaternion[2] , quaternion[3]))



def listener():
    rospy.Subscriber("/cmd_vel", Twist, receive_twist)
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node("car_publisher")
    rospy.set_param("publish_default_positions", True)

    joint_state = JointState()
    joint_state.header = Header()


    trans = TransformStamped()
    trans.header = Header()


    broadcaster = tf.TransformBroadcaster()
    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
