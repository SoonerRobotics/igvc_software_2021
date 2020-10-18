#!/usr/bin/env python

## Lane finder reads in the image topic and pushes out to the lane_deviation topic the current deviation from lane center.

import rospy
import json
from igvc_msgs.msg import motors
from sensor_msgs.msg import Joy


manual_control_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)

def manual_control_callback(data):
    #http://wiki.ros.org/joy#Microsoft_Xbox_360_Wireless_Controller_for_Linux

    axes = data.axes

    drivetrain_msg = motors()
    
    drivetrain_msg.right = axes[4]**3 * 2.2
    if abs(axes[4]) < 0.1:
        drivetrain_msg.right = 0

    drivetrain_msg.left = axes[1]**3 * 2.2
    if abs(axes[1]) < 0.1:
        drivetrain_msg.left = 0

    # rospy.loginfo("manual_control callback.")
    manual_control_pub.publish(drivetrain_msg)

def manual_node():
    rospy.init_node('manual_node', anonymous=True)

    rospy.Subscriber("/joy", Joy, manual_control_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        manual_node()
    except rospy.ROSInterruptException:
        pass