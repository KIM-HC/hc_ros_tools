#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.length = 25
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher("/darknet/edited_dyros",Image,queue_size=5)
    self.image_sub = rospy.Subscriber("/darknet/detection_image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 320 and rows > 240 :
      cv2.line(cv_image, (320+self.length, 240), (320-self.length, 240), (0,0,255), 1)
      cv2.line(cv_image, (320, 240+self.length), (320, 240-self.length), (0,0,255), 1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

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