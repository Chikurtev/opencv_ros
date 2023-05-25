#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
from skimage.metrics import structural_similarity as compare_ssim
import argparse
import imutils
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

path2 = r'/home/denis/catkin_ws/src/cv_basics/scripts/only_table.png'


# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-f", "--first", required=True,
#	help="first input image")
#ap.add_argument("-s", "--second", required=True,
#	help="second input image")
#args = vars(ap.parse_args())

def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
  
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  previous_frame = cv2.imread(path2)
  
  #image cropping
  #print(previous_frame.shape)
  crop_current_frame = current_frame[75:190, 40:312]
  crop_previous_frame = previous_frame[75:190, 40:312]

  # convert the images to grayscale
  grayA = cv2.cvtColor(crop_current_frame, cv2.COLOR_BGR2GRAY)
  grayB = cv2.cvtColor(crop_previous_frame, cv2.COLOR_BGR2GRAY)
  #cv2.imshow("greyA", grayA)
  blurA = cv2.GaussianBlur(grayA,(5,5),0)
  blurB = cv2.GaussianBlur(grayB,(5,5),0)
  #cv2.imshow("gaiusA", blurA)
  # compute the Structural Similarity Index (SSIM) between the two
  # images, ensuring that the difference image is returned
  (score, diff) = compare_ssim(blurA, blurB, full=True)
  diff = (diff * 255).astype("uint8")
  print("SSIM: {}".format(score))
  
  if score > 0.99:
    cv2.imshow("camera", crop_current_frame)
    cv2.imshow("camera0", crop_previous_frame)
    print("the caps are empty")
    cv2.waitKey(1)
  else:

    # threshold the difference image, followed by finding contours to
    # obtain the regions of the two input images that differ
    #thresh = cv2.adaptiveThreshold(diff,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY,11,2)[1]
    thresh = cv2.threshold(diff, 0, 255,
  	    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
  
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours
    for c in cnts:
	  # compute the bounding box of the contour and then draw the
	  # bounding box on both input images to represent where the two
	  # images differ
      (x, y, w, h) = cv2.boundingRect(c)
      cv2.rectangle(crop_current_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
      #cv2.rectangle(previous_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
    # show the output images
    cv2.imshow("Diff", diff)
    cv2.imshow("Thresh", thresh)

    # Display image
    cv2.imshow("camera", crop_current_frame)
    cv2.imshow("camera0", crop_previous_frame)
  
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
