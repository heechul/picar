#!/usr/bin/env python
import roslib;
roslib.load_manifest('car_model')
import cv2, os, rospy, sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from time import sleep

class stream_converter:

    def __init__(self):
            self.webstream = None




if __name__ == '__main__':
    rospy.init_node('stream_converter', anonymous=True)
    rospy.spin()
