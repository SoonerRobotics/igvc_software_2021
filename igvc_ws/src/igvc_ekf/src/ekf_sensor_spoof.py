#!/usr/bin/env python

from numpy import genfromtxt, savetxt, vectorize
from math import degrees
from os.path import expanduser

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3



if __name__ == "__main__":
    # Get home directory
    home = expanduser("~")

    # Load data
    gps_data = genfromtxt(home + "/gps_log.csv", delimiter=',')
    vel_data = genfromtxt(home + "/vel_log.csv", delimiter=',')
    accel_data = genfromtxt(home + "/accel_log.csv", delimiter=',')
    hdg_data = genfromtxt(home + "/hdg_log.csv", delimiter=',')

    # Set the iterator
    i = 0

    # Initialize the node
    rospy.init_node("sensor_spoof_node")

    # Set up publishers
    gps_pub = rospy.Publisher("/igvc/gps", Vector3, queue_size=10)
    vel_pub = rospy.Publisher("/igvc/velocity", Float64, queue_size=10)
    accel_pub = rospy.Publisher("/igvc/acceleration", Float64, queue_size=10)
    hdg_pub = rospy.Publisher("/igvc/heading", Float64, queue_size=10)

    # Define the loop rate
    loop_rate = rospy.Rate(60)

    # Wait for the EKF to connect
    while gps_pub.get_num_connections() == 0:
        pass

    # Run a loop to publish data to the EKF
    while rospy.is_shutdown() == False:
        # GPS
        if i < gps_data.shape[0]:
            gps_msg = Vector3(gps_data[i, 0], gps_data[i, 1], 0)
            gps_pub.publish(gps_msg)

        # Velocity
        if i < vel_data.shape[0]:
            vel_msg = Float64(vel_data[i])
            vel_pub.publish(vel_msg)

        # Acceleration
        if i < accel_data.shape[0]:
            accel_msg = Float64(accel_data[i])
            accel_pub.publish(accel_msg)

        # Heading
        if i < hdg_data.shape[0]:
            hdg_msg = Float64(hdg_data[i, 0])
            hdg_pub.publish(hdg_msg)

        # Increment
        i = i + 1

        # Limit the loop to a certain rate
        loop_rate.sleep()

