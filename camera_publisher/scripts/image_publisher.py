#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('camera_publisher')
import sys
import rospy
import cv2
import numpy
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_publisher:

  def __init__(self):

    self.camera = PiCamera()
    self.camera.resolution = (1024, 768) #(1920, 1080)
    self.camera.ISO = 100
    self.camera.sa = 100
    self.camera.awb = "flash"
    self.camera.co = 100
    self.rawCapture = PiRGBArray(self.camera)
    
    time.sleep(0.1)

    self.image_pub = rospy.Publisher("image_topic",Image)
    self.small_image_pub = rospy.Publisher("small_image_topic",Image)
    self.bridge = CvBridge()

  def publishImage(self):
    for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
        try:
          cv_image = frame.array
        except CvBridgeError as e:
          print(e)
	#cv_image = (cv_image * 0.5.astype(umpy.uint8)
        self.rawCapture.truncate(0)
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
          small = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)
          self.small_image_pub.publish(self.bridge.cv2_to_imgmsg(small, "bgr8"))
        except CvBridgeError as e:
          print("problem")
          print(e)
        #if key == ord('q'):
        #    break

def main(args):
  ip = image_publisher()
  rospy.init_node('image_publisher', anonymous=True)
  try:
    ip.publishImage()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    self.camera.close()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
    main(sys.argv)