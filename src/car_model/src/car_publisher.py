#!/usr/bin/env python

import roslib; roslib.load_manifest('car_model')
import rospy, tf
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


joint_state = None
transf = None
pose_st = None
joint_pub = None
pose_pub = None
broadcaster = None


def receive_twist(msg):
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


    f_left = 0
    f_right = 0
    back_wheels = 0

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


        joint_state.position = [ 0, 0, f_left, f_right, back_wheels, back_wheels ]



    # for now, the following section is just dummy code

    else: # strafe

        if msg.linear.x > 0: # if forward

            if msg.angular.z > 0: #
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z]

            elif msg.angular.z == 0: #
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: # o
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z]

        elif msg.linear.x == 0: # if angle

            if msg.angular.z > 0: #
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z]

            elif msg.angular.z == 0: #
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: # o
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z]

        else: # if reverse

            if msg.angular.z > 0: #
                joint_state.position = [ 0.785, 0.785, 0.785, 0.785, 3.14*msg.angular.z, 3.14*msg.angular.z]

            elif msg.angular.z == 0: #
                joint_state.position = [ 0, 0, 0, 0, 0, 0 ]

            else: #
                joint_state.position = [ -0.383, -0.383, -0.383, -0.383, -3.14*msg.angular.z, -3.14*msg.angular.z]

    # end of dummy code section

    move_x = 0
    move_y = 0
    move_z = 0



    # update transform and pose
    quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.angular.z)


    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = "joint_states"
    trans.child_frame_id = joint_state.name
    trans.transform.translation.x += msg.linear.x
    trans.transform.translation.y += msg.linear.y
    trans.transform.translation.z += msg.linear.z
    trans.transform.rotation = quaternion

    """
    pose_st.header.stamp = rospy.Time.now()
    pose_st.header.frame_id = "world"
    pose_st.pose.position.x += msg.linear.y
    pose_st.pose.position.y += msg.linear.x
    pose_st.pose.position.z += msg.linear.z
    pose_st.pose.orientation.x += quaternion[0]
    pose_st.pose.orientation.y += quaternion[1]
    pose_st.pose.orientation.z += quaternion[2]
    pose_st.pose.orientation.w += quaternion[3]
    """




    # send the joint state and transform
    joint_pub.publish(joint_state)
    #pose_pub.publish(pose_st)

    """
    broadcaster.sendTransform(
        (msg.linear.x, msg.linear.y, msg.linear.z),
        trans.transform.rotation,
        rospy.Time.now(),
        trans.header.frame_id,
        trans.child_frame_id
    )
    broadcaster.sendTransform(
        (msg.linear.x, msg.linear.y, msg.angular.z),
        (pose_st.pose.orientation.x, pose_st.pose.orientation.y, pose_st.pose.orientation.z, pose_st.pose.orientation.w),
        rospy.Time.now(),
        "base_link",
        "world"
    )
    """



def listener():
    rospy.init_node("joint_state_publisher")
    rospy.Subscriber("/cmd_vel", Twist, receive_twist)

    rospy.spin()

if __name__ == '__main__':
    joint_state = JointState()
    joint_state.header = Header()

    trans = TransformStamped()
    trans.header = Header()

    """
    pose_st = PoseStamped()
    pose_st.header = Header()
    """

    broadcaster = tf.TransformBroadcaster()
    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    #pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
