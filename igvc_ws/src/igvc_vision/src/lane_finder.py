#!/usr/bin/env python

## Lane finder reads in the image topic and pushes out to the lane_deviation topic the current deviation from lane center.

import rospy
import cv2
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
image_pub = rospy.Publisher("/igvc/road_vision",Image)

def camera_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    dims = (128, 128)
    blur_amount = 5

    image = cv2.resize(cv_image, dims)
    image = cv2.medianBlur(image, blur_amount)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    xs = []
    ys = []

    for x in range(dims[0]):
        for y in range(dims[1]):
            if hsv_image[y,x][1] < 50 and hsv_image[y,x][2] > 100:
                xs.append(x)
                ys.append(y)
                image[y,x] = (180,0,0)

    xavg = sum(xs)/len(xs)
    yavg = sum(ys)/len(ys)

    xavg += math.exp(yavg - dims[1]) # if we are running into not good stuff, just start turning lol

    xavg = int(xavg)
    yavg = int(yavg)

    rospy.loginfo("I saw a camera frame" + str(math.atan2(xavg - dims[0]/2, dims[1] - yavg)))
    cv2.line(image, (dims[0]/2, dims[1]), (xavg, yavg), (0,255,0), 4)

    image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    image_pub.publish(image_message)

def lane_finder():
    rospy.init_node('lane_finder', anonymous=True)

    # pub = rospy.Publisher('/igvc/lane_deviation', Float32, queue_size=1)
    rospy.Subscriber("/cv_camera/image_raw", Image, camera_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        lane_finder()
    except rospy.ROSInterruptException:
        pass