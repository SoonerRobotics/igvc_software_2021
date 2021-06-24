#!/usr/bin/env python3

import rospy
from ekf import EKF
from igvc_msgs.msg import gps, velocity, EKFState
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import numpy as np

np.set_printoptions(precision=4)

# period
dt = None
# mobility start/stop flag
mobi_start = False
# publisher
state_pub = None
# the ekf object
ekf = None

# NOTE simulate a mobi_start artificially for testing after receiving 5 messages.
TEMP_counter = 0

# run the EKF.
def timer_callback(event):
    global ekf
    ekf.predict()
    ekf.measure()
    ekf.update()
    state_pub.publish(ekf.get_state_msg(mobi_start))

# functions to receive sensor readings.
def meas_gps(gps_msg):
    global ekf, TEMP_counter
    if not mobi_start or TEMP_counter < 5:
        TEMP_counter += 1
        print("setting start GPS")
        ekf.set_start_gps(gps_msg)
    else:
        print("setting current GPS")
        ekf.measure_gps(gps_msg)
def meas_imu(imu_msg):
    global ekf
    print("setting IMU meas")
    ekf.measure_imu(imu_msg)
def meas_vel(vel_msg):
    global ekf
    print("setting velocity meas")
    ekf.measure_velocities(vel_msg)
def init_mobi_start(mobi_msg):
    global mobi_start
    # NOTE one-way state transition
    mobi_start = True

def main():
    global state_pub, ekf, dt
    # initalize the node in ROS
    rospy.init_node('ekf_node')

    # initialize the EKF
    dt = 0.02
    # 8D process noise matrix
    Q = np.matrix([
        [1.5,0,0,0,0,0,0,0],
        [0,1.5,0,0,0,0,0,0],
        [0,0,1.5,0,0,0,0,0],
        [0,0,0,1.5,0,0,0,0],
        [0,0,0,0,1.5,0,0,0],
        [0,0,0,0,0,1.5,0,0],
        [0,0,0,0,0,0,1.5,0],
        [0,0,0,0,0,0,0,1.5]])
    # 6D measurement uncertainty matrix
    R = np.matrix([
        [100000,0,0,0,0,0],
        [0,10000,0,0,0,0],
        [0,0,10,0,0,0],
        [0,0,0,10,0,0],
        [0,0,0,0,0.005,0],
        [0,0,0,0,0,0.005]])
    ekf = EKF(dt = dt, Q=Q, R=R)

    ## Subscribe to Mobility Start/Stop messages
    rospy.Subscriber("/igvc/mobstart", Bool, init_mobi_start, queue_size=1)
    ## Subscribe to Sensor Values
    rospy.Subscriber("/igvc/gps", gps, meas_gps, queue_size=1)
    # NOTE for competition, topic was "/imu/"
    rospy.Subscriber("/igvc/imu", Imu, meas_imu, queue_size=1)
    rospy.Subscriber("/igvc/velocity", velocity, meas_vel, queue_size=1)
    ## Publish the KF state
    state_pub = rospy.Publisher("/igvc/state", EKFState, queue_size=1)
    
    # create timer with a period of dt seconds (1/dt Hz)
    rospy.Timer(rospy.Duration(dt), timer_callback)
    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
