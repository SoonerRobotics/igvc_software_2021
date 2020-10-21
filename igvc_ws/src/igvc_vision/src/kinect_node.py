#!/usr/bin/env python

import rospy
import json
import freenect
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ROS node that receives RGB image and depth image from XBOX Kinect camera
# and publishes them as image message

bridge = CvBridge()

# Receives color image from the Kinect and publishes it as a image message
def get_rgb(timer_event):
    global kinect_rgb_pub
    
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

    image_msg = bridge.cv2_to_imgmsg(array)
    kinect_rgb_pub.publish(image_msg)

# Receives depth image from the Kinect and publishes it as a image message
def get_depth(timer_event):
    global kinect_depth_pub

    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    
    image_msg = bridge.cv2_to_imgmsg(array)
    kinect_depth_pub.publish(image_msg)
    

# Initialize the Kinect node as well as instantiate necessary publishers
def init_kinect_node():

    # Setup Kinect node
    rospy.init_node("kinect_node", anonymous = False)

    # Instantiate publishers for color image and depth image
    kinect_rgb_pub = rospy.Publisher('/igvc/kinect_rbg/', Image, queue_size = 10)
    kinect_depth_pub = rospy.Publisher('/igvc/kinect_depth/', Image, queue_size = 10)

    rospy.Timer(rospy.Duration(secs = 0.03), get_rgb)
    rospy.Timer(rospy.Duration(secs = 0.03), get_depth)

    rospy.spin()

if __name__ == '__main__':
    try:
        init_kinect_node()
    except rospy.ROSInterruptException:
        pass