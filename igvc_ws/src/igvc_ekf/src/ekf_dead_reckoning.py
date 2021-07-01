#!/usr/bin/env python3

import rospy
from igvc_msgs.msg import gps, velocity, motors, EKFState, imuodom
from sensor_msgs.msg import Imu
from tf import transformations
from math import sin, cos
import numpy as np

np.set_printoptions(precision=4)

## Robot Characteristics
WHEEL_RADIUS = 0.127
WHEELBASE_LEN = 0.76 # TODO verify this

## Constants
I_8 = np.matrix([ # 8D identity matrix
    [1,0,0,0,0,0,0,0],
    [0,1,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0],
    [0,0,0,1,0,0,0,0],
    [0,0,0,0,1,0,0,0],
    [0,0,0,0,0,1,0,0],
    [0,0,0,0,0,0,1,0],
    [0,0,0,0,0,0,0,1]])
I_4 = np.matrix([ # 4D identity matrix
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]])
# lat_to_m = 110944.33
# lon_to_m = 91058.93

## Kalman Filter variables
dt = 0.1 # period in seconds
X = None # current state
X_next = None # prediction for next
F = None # state transition matrix (dynamic model)
F_trans = None
P = None # current covariance
P_next = None # prediction for next
Z = None # measurements
Z_buffer = [0,0,0,0] # holds measurements as they come in (outside the measure stage)
H = None # observation matrix (dynamic model)
H_trans = None
Q = None # process noise
R = None # meas uncertainty

# init flag
initialized = False

## Publishers
state_pub = None

## Kalman Filter functions
def initialize():
    global initialized, Q, R, X, X_next, F, F_trans, Z, H, H_trans, P, P_next
    ## Set uncertainties
    Q = np.multiply(1.1,I_8)
    #R = np.multiply(0.1,I_4)
    R = np.matrix([
        [0.0017,0,0,0],
        [0,0.001,0,0],
        [0,0,0.005,0],
        [0,0,0,0.005]])

    ## Initialize the state
    # we will track 8 things: (x,xdot,y,ydot,phi,phidot,v_l,v_r)
    X = np.transpose(np.matrix([0,0,0,0,0,0,0,0]))
    X_next = np.transpose(np.matrix([0,0,0,0,0,0,0,0]))

    ## Initialize the state transition matrix
    # will need to update the trig entries each time. for now use a temp value.
    cos_phi, sin_phi = 0.1, 0.1
    F = np.matrix([
        [1,dt,0,0,0,0,0,0],
        [0,0,0,0,0,0,0.5*WHEEL_RADIUS*cos_phi,0.5*WHEEL_RADIUS*cos_phi],
        [0,0,1,dt,0,0,0,0],
        [0,0,0,0,0,0,0.5*WHEEL_RADIUS*sin_phi,0.5*WHEEL_RADIUS*sin_phi],
        [0,0,0,0,1,dt,0,0],
        [0,0,0,0,0,0,WHEEL_RADIUS/WHEELBASE_LEN,-WHEEL_RADIUS/WHEELBASE_LEN],
        [0,0,0,0,0,0,1,0],
        [0,0,0,0,0,0,0,1]])
    F_trans = np.transpose(F)

    ## Initialize the measurements matrix
    # tracks 4 things: (yaw,yaw_rate,v_l,v_r)
    # we can add GPS optionally later
    Z = np.transpose(np.matrix([0,0,0,0]))

    ## Initialize the observation matrix
    H = np.matrix([
        [0,0,0,0,1,0,0,0],
        [0,0,0,0,0,1,0,0],
        [0,0,0,0,0,0,1,0],
        [0,0,0,0,0,0,0,1]])
    H_trans = np.transpose(H)

    ## Initialize the covariance matrix with temporary values for the useful entries
    P = np.multiply(0.1,I_8)
    # P = np.matrix([
    #     [1,dt,0,0,0,0,0,0],
    #     [0,0,0,0,-0.1,0,0.1,0.1],
    #     [0,0,1,dt,0,0,0,0],
    #     [0,0,0,0,0.1,0,0.1,0.1],
    #     [0,0,0,0,1,dt,0,0],
    #     [0,0,0,0,0,0,WHEEL_RADIUS/WHEELBASE_LEN,WHEEL_RADIUS/WHEELBASE_LEN],
    #     [0,0,0,0,0,0,1,0],
    #     [0,0,0,0,0,0,0,1]])
    P_next = P

    ## set initialized flag
    initialized = True
    print("initialized KF")

def predict():
    global X_next, P_next, F, F_trans
    ## Update the state transition matrix, F and F_trans
    cos_phi = cos(X[4])
    sin_phi = sin(X[4])
    F[1,6] = 0.5*WHEEL_RADIUS*cos_phi
    F[1,7] = 0.5*WHEEL_RADIUS*cos_phi
    F[3,6] = 0.5*WHEEL_RADIUS*sin_phi
    F[3,7] = 0.5*WHEEL_RADIUS*sin_phi
    # update transpose entries too
    F_trans[6,1] = 0.5*WHEEL_RADIUS*cos_phi
    F_trans[7,1] = 0.5*WHEEL_RADIUS*cos_phi
    F_trans[6,3] = 0.5*WHEEL_RADIUS*sin_phi
    F_trans[7,3] = 0.5*WHEEL_RADIUS*sin_phi

    ## Calculate predicted state for next iteration using dynamic model
    # state extrapolation: X(n+1) = F*X(n) + w (ignore process noise)
    X_next = np.matmul(F,X)
    #print("Prediction:\n", X_next)

    ## Extrapolate the estimate uncertainty
    # covariance extrapolation: P(n+1) = F*P(n)*F^T + Q
    P_next = np.matmul(np.matmul(F,P),F_trans) + Q

def measure():
    global Z
    ## Input measurement uncertainty
    # would update R, but we assume it's constant

    ## Load measured values from the buffer
    Z = np.transpose(np.matrix([Z_buffer[0],Z_buffer[1],Z_buffer[2],Z_buffer[3]]))
    #print("Measurement:\n", Z)

def update():
    global K, X, P
    ## Calculate kalman gain
    # compute innovation covariance: S = H*P*H^T + R
    S = np.matmul(np.matmul(H,P),H_trans) + R #4x4 = 4x8 * 8x8 * 8x4 + 4x4
    #print("S:\n", S)
    # optimal kalman gain: K = P*H^T*S^-1
    S_inv = np.linalg.pinv(S) #4x4
    K = np.multiply(np.matmul(np.matmul(P,H_trans),S_inv), H_trans) #8x4 = 8x8 * 8x4 * 4x4
    #print("Kalman Gain:\n", K)

    ## Estimate the current state using the state update equation
    # state update: X(n+1) = X(n) + K*(Z-H*X(n))
    X = X_next + np.matmul(K,(Z-np.matmul(H,X_next))) #8x1 = 8x1 + 8x4 * (4x1 - 4x8 * 8x1)
    #print("State:\n", X)
    
    ## Update the current estimate uncertainty
    # covariance update: P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
    # or P = (I-K*H)*P (simple version)
    P = np.matmul((I_8-np.matmul(K,H)),P_next) #8x8 = (8x8 - 8x4 * 4x8) * 8x8
    #P = np.matmul(np.matmul((I_8-np.matmul(K,H)),P_next),np.transpose(I_8-np.matmul(K,H))) + np.matmul(np.matmul(K,R),np.transpose(K))
    #print("Covariance:\n", P)

## Run the KF
def timer_callback(event):
    if not initialized:
        initialize()
    else:
        predict()
        measure()
        update()
        ## Publish the state for the robot to use.
        state_msg = EKFState()
        state_msg.x = X[0]
        state_msg.x_velocity = X[1]
        state_msg.y = X[2]
        state_msg.y_velocity = X[3]
        state_msg.yaw = X[4]
        state_msg.yaw_rate = X[5]
        state_msg.left_velocity = X[6]
        state_msg.right_velocity = X[7]
        state_pub.publish(state_msg)

# ## print state vector to the console in a readable format
# def print_state(name, vector):
#     state = mat_to_ls(vector)
#     #print(state)
#     line = name + ": x=" + "{:.2f}".format(state[0]) + ", y=" + "{:.2f}".format(state[1]) \
#         + ", x-vel=" + "{:.2f}".format(state[2]) + ", y-vel=" + "{:.2f}".format(state[3])
#     print(line)

## Functions to receive sensor readings.
## Stay in buffer until measure() is run each clock cycle.
def meas_imu(imu_msg):
    global Z_buffer
    # saves robot's current heading in radians (0 north, CW)
    orientation = imu_msg.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    # do bad stuff to make the simulator work
    bad_quat = [-quaternion[3], quaternion[1], -quaternion[2], quaternion[0]]
    #print(transformations.euler_from_quaternion(bad_quat))
    yaw_rads = transformations.euler_from_quaternion(bad_quat)[2] #TODO check sign #yaw should be 2 but hmmm
    Z_buffer[0] = yaw_rads
    yaw_rate = -imu_msg.angular_velocity.z #TODO check sign
    Z_buffer[1] = yaw_rate
    
def meas_vel(vel_msg):
    global Z_buffer
    Z_buffer[2] = vel_msg.leftVel
    Z_buffer[3] = vel_msg.rightVel

# def meas_gps(gps_msg):
#     global Z_buffer
#     if gps_msg.hasSignal:
#         Z_buffer[4] = gps_msg.latitude
#         Z_buffer[5] = gps_msg.longitude
#     else:
#         Z_buffer[4] = None
#         Z_buffer[5] = None

def main():
    global state_pub
    # initalize the node in ROS
    rospy.init_node('ekf_dead_reckoning')

    ## Subscribe to Sensor Values
    rospy.Subscriber("/igvc/imu", Imu, meas_imu, queue_size=1)
    rospy.Subscriber("/igvc/velocity", velocity, meas_vel, queue_size=1)
    #rospy.Subscriber("/igvc/gps", Gps, meas_gps, queue_size=1)
    ## Subscribe to Control Parameters
    #rospy.Subscriber("/igvc/motors_raw", motors, update_control_signal, queue_size=1)

    ## Publish the KF state
    state_pub = rospy.Publisher("/igvc/state", EKFState, queue_size=1)
    
    # create timer with a period of 0.1 (10 Hz)
    rospy.Timer(rospy.Duration(dt), timer_callback)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
