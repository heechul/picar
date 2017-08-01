#!/usr/bin/env python
import roslib; roslib.load_manifest('picar_base')
import csv, cv2, model, os, params, preprocess, rospy, signal, sys, time
import local_common as cm; import tensorflow as tf;
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from httplib import BadStatusLine
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from threading import Thread

#Stop source warnings from appearing
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


class VideoDriver:

    def __init__(self, video_location):
        self.bridge = CvBridge()
        self.video_location = video_location
        self.video_source = cv2.VideoCapture(video_location)
        self.twist_pub = rospy.Publisher('/Self_Driver/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/sensor_msgs/Image', Image, queue_size=1)


        # publish the video frame for viewing in RViz, etc.
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

        try:
            speed = 0.5
            frame_count = cm.frame_count(self.video_location)
            tempAngle = 0 #Hold the angle
            turned_yet = False
            angz = 0

            #Open the model
            sess = tf.InteractiveSession()
            saver = tf.train.Saver()
            model_name = 'model.ckpt'
            model_path = cm.jn(params.save_dir, model_name)
            saver.restore(sess, model_path)

            #Get the predicted angle from the model for each frame and publish the angle
            for frame_id in xrange(frame_count):
                ret, img = self.video_source.read() #Get the frame
                if not ret: #Make sure the frame exists
                    return

                self.publish_frame(img) #Publish the frame for viewing in RViz

                img = preprocess.preprocess(img) #Process the image
                deg = model.y.eval(feed_dict={model.x: [img], model.keep_prob: 1.0})[0][0] #Predict the angle
                deg = round(deg * 8) / 8 #Round the angle to the nearest eighth


                old_angz = angz
                angz = (tempAngle - deg) * 1.5
                if (deg < tempAngle and deg < -tempAngle) or \
                   (deg > tempAngle and deg > -tempAngle):
                    angz = old_angz

                #Create a twist message that determines if a turn is necessary and publish it
                twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angz;
                self.twist_pub.publish(twist)

                #Update the temporary angle value to the angle of the current frame
                tempAngle = deg
                time.sleep(0.06)
            """end of code section"""


        except:
            return

        finally:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
            self.twist_pub.publish(twist)
            self.video_source.release()
            time.sleep(5)
            rospy.signal_shutdown("Shutting down")

def signal_handler(signal, frame):
    cv2.destroyAllWindows()
    rospy.signal_shutdown("Shutting down")
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('video_driver', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    simulator = VideoDriver("../datasets/dataset4/out-mencoder.avi")
    simulator.publish_to_car()
    rospy.spin()
