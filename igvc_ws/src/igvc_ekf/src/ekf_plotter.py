#!/usr/bin/env python
import numpy as np
from math import degrees
from matplotlib import pyplot as plt

import rospy
from igvc_msgs.msg import EKFState, EKFConvergence
from geometry_msgs.msg import Pose
from roslib.packages import get_pkg_dir

pkg_path = get_pkg_dir("igvc_ekf")

true_pose_csv_data = ""
ekf_pose_csv_data = ""

def plot_x(msg):
    global counter
    if counter % 10 == 0:
        # plt.figure(1)
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        # plt.plot(msg.x, msg.y, '*')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.00000000001)

        csv_file = open(pkg_path + "/data/export.csv", "a")
        state_str = ""
        state_str += str(counter/10) + ","
        state_str += str(degrees(msg.latitude)) + ","
        state_str += str(degrees(msg.longitude)) + ","
        state_str += str(msg.global_heading) + ","
        state_str += str(msg.x) + ","
        state_str += str(msg.y) + ","
        state_str += str(msg.yaw) + ","
        #state_str += str(msg.velocity) + ","
        #state_str += str(msg.yaw_rate) + ","
        #state_str += str(msg.left_angular_vel) + ","
        #state_str += str(msg.right_angular_vel) + ","
        #state_str += str(msg.acceleration) + ","

        csv_file.write(state_str + "\n")
        csv_file.close()

    counter += 1

def plot_conv(msg):
    global counter
    if counter % 10 == 0:
        # stamp = msg.header.stamp
        # plt.figure(2)
        # time = stamp.secs + stamp.nsecs * 1e-9
        # plt.plot(time, msg.data, '*')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.00000000001)

        conv_file = open(pkg_path + "/data/convergence.csv", "a")
        state_str = str(msg.data) + "\n"
        conv_file.write(state_str)
        conv_file.close()

def plot_ground_truth(pose):
    global true_counter
    if true_counter % 10 == 0:
        pose_file = open(pkg_path + "/data/ground_truth.csv", "a")
        state_str = str(pose.position.x) + ", " + str(pose.position.y) + "\n"
        pose_file.write(state_str)
        pose_file.close()

    true_counter += 1

if __name__ == '__main__':
    counter = 0
    true_counter = 0

    rospy.init_node("plotter")

    rospy.Subscriber("/igvc_ekf/filter_output", EKFState, plot_x)
    rospy.Subscriber("/sim/true_pose", Pose, plot_ground_truth)
    rospy.Subscriber("/igvc_ekf/EKFConvergence", EKFConvergence, plot_conv)

    # EKF logging
    csv_file = open(pkg_path + "/data/export.csv", "w")
    #csv_file.write("id, lat, lon, theta, x, y, psi, vel, psi_dot, left_v, right_v, accel\n")
    csv_file.close()

    # EKF convergence
    csv_file = open(pkg_path + "/data/convergence.csv", "w")
    csv_file.close()

    # Ground truth pose
    csv_file = open(pkg_path + "/data/ground_truth.csv", "w")
    csv_file.close()

    # plt.ion()
    # plt.show()
    rospy.spin()