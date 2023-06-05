#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

OPENCV_MAJOR_VERSION = int(cv2.__version__.split('.')[0])

bg_subtractor = cv2.createBackgroundSubtractorKNN(detectShadows=True)

erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 5))
dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 11))

def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
   
  # Display image
  cv2.imshow("camera", current_frame)
  
  fg_mask = bg_subtractor.apply(current_frame)

  _, thresh = cv2.threshold(fg_mask, 244, 255, cv2.THRESH_BINARY)
  cv2.erode(thresh, erode_kernel, thresh, iterations=2)
  cv2.dilate(thresh, dilate_kernel, thresh, iterations=2)

  if OPENCV_MAJOR_VERSION >= 4:
      # OpenCV 4 or a later version is being used.
      contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
  else:
        # OpenCV 3 or an earlier version is being used.
        # cv2.findContours has an extra return value.
        # The extra return value is the thresholded image, which is
        # unchanged, so we can ignore it.
      _, contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)

  for c in contours:
      if cv2.contourArea(c) > 1000:
          x, y, w, h = cv2.boundingRect(c)
          cv2.rectangle(current_frame, (x, y), (x+w, y+h), (255, 255, 0), 2)

  cv2.imshow('knn', fg_mask)
  #cv2.imshow('thresh', thresh)
  #cv2.imshow('detection', current_frame)

  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/table/camera1/image_raw', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
