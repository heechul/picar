#!/usr/bin/env python
import roslib;
roslib.load_manifest('car_model')
import cv2, os, rospy, socket, sys, urllib2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from time import sleep

class stream_converter:

    def __init__(self):
        self.webstream = urllib2.urlopen("http://devboard-picar-wifi.ittc.ku.edu:8080")
        self.bridge = CvBridge()
        self.frame_pub = rospy.Publisher("/sensor_msgs/Image",Image, queue_size=50)
        self.camera = cv2.VideoCapture(0)

    def stream_to_images(self):

        try:
            success, frame = self.webstream.read(360)

            while success:
                pub_frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                pub_frame.header = Header()
                pub_frame.header.frame_id = "odom"
                pub_frame.header.stamp = rospy.Time.now()

                try:
                    self.frame_pub.publish(pub_frame)
                except CvBridgeError as e:
                    rospy.loginfo(e)

                success, frame = self.webstream.read(360)
                sleep(0.03)

        except KeyboardInterrupt:
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Shutting down")
            sys.exit(0)

def main(args):
    sc = stream_converter()
    try:
        sc.stream_to_images()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down")
        sys.exit(0)
    rospy.signal_shutdown("Shutting down")


if __name__ == '__main__':
    rospy.init_node('stream_converter', anonymous=True)
    main(sys.argv)
    rospy.spin()
