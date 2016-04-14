#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('camera_publisher')
import sys
import rospy
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_publisher:

  def __init__(self):

    self.camera = PiCamera()
    self.camera.resolution = (1024, 768)

    self.rawCapture = PiRGBArray(self.camera)
    
    time.sleep(0.1)

    self.image_pub = rospy.Publisher("image_topic",Image)

    self.bridge = CvBridge()

  def publishImage(self):
    for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
        try:
          cv_image = frame.array
        except CvBridgeError as e:
          print(e)
    
        #cv2.imshow("Image window", cv_image)
        #key = cv2.waitKey(1) & 0xFF

        self.rawCapture.truncate(0)
        try:
          cropped = cv_image[1:650, 290:950] #[1:900, 330:1275] for large image [1:450, 150:600] # these values work for the 1024x768 resolution
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cropped, "bgr8"))
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
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)