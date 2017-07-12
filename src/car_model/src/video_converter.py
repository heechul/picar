#!/usr/bin/env python
import roslib;
roslib.load_manifest('car_model')
import cv2, os, rospy, sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep

#from __future__ import print_function

class video_converter:

    def __init__(self):
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("sensor_msgs/Image",Image, queue_size=50)
        #self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
        self.test_video = cv2.VideoCapture("example.avi")



    def vid_to_images(self):

        success, image = self.test_video.read()
        while success:

            #cv2.imshow("Image window", image)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))
            except CvBridgeError as e:
                rospy.loginfo(e)

            #cv2.imwrite("frames/frame%d.jpg"%count, image)
            success, image = self.test_video.read()

            sleep(0.05)

        rospy.loginfo("Done")
        rospy.signal_shutdown("Shutting down")
        #os.system("rm frames/*")




def main(args):
    vc = video_converter()
    rospy.init_node('video_converter', anonymous=True)
    try:
        vc.vid_to_images()
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
