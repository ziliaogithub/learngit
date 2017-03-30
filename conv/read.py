#!/usr/bin/env python

import sys
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class image_converter:

	def __init__(self):
	  self.bridge = CvBridge()
	  self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
          self.image_pub = rospy.Publisher ('/ardrone/image_gray',Image)

	def callback(self,data):
	  cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
          gray = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
          cv_gray = self.bridge.cv2_to_imgmsg(gray,"mono8")
	  self.image_pub.publish(cv_gray)
	  cv2.imshow("Image window", gray)
          cv2.waitKey(2)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
