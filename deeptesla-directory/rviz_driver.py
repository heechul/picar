#!/usr/bin/env python
import roslib; roslib.load_manifest('picar_base')
import csv, cv2, model, os, params, preprocess, rospy, signal, sys, time
import local_common as cm; import tensorflow as tf;
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from httplib import BadStatusLine
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from threading import Thread


class driver_simulation:

    def __init__(self, video_location):
        self.bridge = CvBridge()
        self.video_location = video_location
        self.video_source = cv2.VideoCapture(video_location)
        self.stream_source = "http://devboard-picar-wifi:8080?action=snapshot"
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)
        self.image_pub = rospy.Publisher('/sensor_msgs/Image', Image, queue_size=50)


    def get_stream_snapshot(self):
        try:
            snapshot = urllib2.urlopen(self.stream_source)
            frame = np.asarray(bytearray(snapshot.read()), dtype="uint8")
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            pub_frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            pub_frame.header = Header()
            pub_frame.header.frame_id = "odom"
            pub_frame.header.stamp = rospy.Time.now()

            try:
                self.frame_pub.publish(pub_frame)
            except CvBridgeError as e:
                rospy.loginfo(e)

        except (KeyboardInterrupt, urllib2.URLError, BadStatusLine) as e:
            rospy.loginfo(e)
            return


    def publish_frame(self, frame):
        try:
            pub_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
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
        rospy.loginfo("function beginning")

        try:
            rospy.loginfo("try")
            """adapted from angle_publisher.py"""
            speed = 2
            frame_count = cm.frame_count(self.video_location)
            tempAngle = 0 #Hold the angle

            #Open the model
            sess = tf.InteractiveSession()
            saver = tf.train.Saver()
            model_name = 'model.ckpt'
            model_path = cm.jn(params.save_dir, model_name)
            saver.restore(sess, model_path)
            rospy.loginfo("model")

            #Get the predicted angle from the model for each frame and publish the angle
            for frame_id in xrange(frame_count):
                rospy.loginfo("loop")
                ret, img = self.video_source.read() #Get the frame
                self.publish_frame(img)

                assert ret #Make sure the frame exists
                img = preprocess.preprocess(img) #Process the image
                deg = self.model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
                deg = round(deg * 8) / 8 #Round the angle to the nearest eighth

                #Create a twist message that determines if a turn is necessary and publish it
                twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = (tempAngle - deg) * 2
                self.twist_pub.publish(twist)

                #Update the temporary angle value to the angle of the current frame
                tempAngle = deg
                rospy.loginfo("iter")
                #sleep(0.03)
            """end of code section"""

        except:
            return

        finally:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
            self.twist_pub.publish(twist)


def signal_handler(signal, frame):
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('driver_simulation', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    simulator = None
    try:
        video_source = sys.argv[1]
        simulator = driver_simulation(video_source)
    except (IndexError, ValueError):
        simulator = driver_simulation("../datasets/dataset23/out-mencoder.avi")
        simulator.publish_to_car()
    rospy.spin()
