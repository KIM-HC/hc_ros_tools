#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_publisher:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=5)
    self.bridge = CvBridge()
    self.image_path = "data/img"
    self.num_img = 10000  # total number of images

  def publish_image(self):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  im_pub = image_publisher()
  rospy.init_node('image_publisher', anonymous=True)
  im_pub.publish_image()


  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)