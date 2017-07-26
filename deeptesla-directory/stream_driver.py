#!/usr/bin/env python
import roslib; roslib.load_manifest('picar_base')
import csv, cv2, model, os, params, preprocess, rospy, socket, signal, sys, time, urllib2
import local_common as cm; import tensorflow as tf; import numpy as np;
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from httplib import BadStatusLine
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from threading import Thread

#Stop source warnings from appearing
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


class StreamDriver:

    def __init__(self):
        self.bridge = CvBridge()
        self.stream_source = "http://devboard-picar-wifi:8080?action=snapshot"
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/sensor_msgs/Image', Image, queue_size=1)


    def publish_snapshot(self, snapshot):
        try:
            pub_image = self.bridge.cv2_to_imgmsg(snapshot, "bgr8")
            pub_image.header = Header()
            pub_image.header.frame_id = "odom"
            pub_image.header.stamp = rospy.Time.now()

            try:
                self.image_pub.publish(pub_image)
            except CvBridgeError as e:
                rospy.loginfo(e)

        except KeyboardInterrupt:
            return


    def publish_to_car(self):
        twist = Twist()

        try:
            """adapted from angle_publisher.py"""
            speed = 2
            tempAngle = 0 #Hold the angle

            #Open the model
            """
            sess = tf.InteractiveSession()
            saver = tf.train.Saver()
            model_name = 'model.ckpt'
            model_path = cm.jn(params.save_dir, model_name)
            saver.restore(sess, model_path)
            """

            while True:
                snapshot = urllib2.urlopen(self.stream_source)
                original_frame = snapshot.read()
                #assert frame #Make sure the frame exists

                frame = np.asarray(bytearray(original_frame), dtype="uint8")
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                self.publish_snapshot(frame) #Publish the frame for viewing in RViz


                img = preprocess.preprocess(img) #Process the image
                deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
                deg = round(deg * 8) / 8 #Round the angle to the nearest eighth

                #Create a twist message that determines if a turn is necessary and publish it
                twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = (tempAngle - deg) * 2;
                self.twist_pub.publish(twist)

                #Update the temporary angle value to the angle of the current frame
                tempAngle = deg
                time.sleep(0.06)

            """end of code section"""


        except (KeyboardInterrupt, urllib2.URLError, BadStatusLine) as e:
            rospy.loginfo(e)
            return

        finally:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
            self.twist_pub.publish(twist)
            rospy.signal_shutdown("Shutting down")
            sys.exit(0)


def signal_handler(signal, frame):
    cv2.destroyAllWindows()
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)


if __name__ == "__main__":
    rospy.init_node('stream_driver', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    simulator = StreamDriver()
    simulator.publish_to_car()
    rospy.spin()
