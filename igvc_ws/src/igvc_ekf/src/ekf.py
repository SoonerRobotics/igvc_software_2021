#!/usr/bin/env python3

from igvc_msgs.msg import gps, velocity, EKFState
from sensor_msgs.msg import Imu
from tf import transformations
from math import sin, cos
import numpy as np
np.set_printoptions(precision=4)

# for symbolic jacobian calculation
# NOTE you must install sympy for this part to work, 
# but it is not necessary except when deriving the initial EKF.
# from sympy import sin, cos, Matrix, Symbol
# from sympy.abc import phi, x, y, t, r, l
# from sympy import pprint

# robot characteristics
WHEEL_RADIUS = 0.127
WHEELBASE_LEN = 0.76
# identity matrices
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
# coords of Oakland University: 42.6679° N, 83.2082° W.
# conversions calculated with http://www.csgnetwork.com/degreelenllavcalc.html.
LAT_TO_M = 111086.33 #linear conversion latitude to meters
LON_TO_M = 81972.46 #linear conversion longitude to meters

class EKF:
    def __init__(self, dt:float, Q=None, R=None):
        ## Set timestep size
        self.dt = dt

        ## Set uncertainties
        if Q is not None:
            self.Q = Q
        else: #default
            self.Q = np.multiply(1.5,I_8)
        if R is not None:
            self.R = R
        else: #default from the competition
            self.R = np.matrix([
                [100000,0,0,0,0,0],
                [0,10000,0,0,0,0],
                [0,0,10,0,0,0],
                [0,0,0,10,0,0],
                [0,0,0,0,0.005,0],
                [0,0,0,0,0,0.005]])

        ## Initialize the state
        # we will track 8 things: (x,xdot,y,ydot,phi,phidot,v_l,v_r)
        self.X = np.transpose(np.matrix([0,0,0,0,0,0,0,0]))
        self.X_next = np.transpose(np.matrix([0,0,0,0,0,0,0,0]))

        ## Initialize the state transition matrix
        # will need to update the trig entries each time. for now use a temp value.
        cos_phi, sin_phi = 0.1, 0.1
        self.F = np.matrix([
            [1,dt,0,0,0,0,0,0],
            [0,0,0,0,0,0,0.5*WHEEL_RADIUS*cos_phi,0.5*WHEEL_RADIUS*cos_phi],
            [0,0,1,dt,0,0,0,0],
            [0,0,0,0,0,0,0.5*WHEEL_RADIUS*sin_phi,0.5*WHEEL_RADIUS*sin_phi],
            [0,0,0,0,1,dt,0,0],
            [0,0,0,0,0,0,WHEEL_RADIUS/WHEELBASE_LEN,-WHEEL_RADIUS/WHEELBASE_LEN],
            [0,0,0,0,0,0,1,0],
            [0,0,0,0,0,0,0,1]])
        self.F_trans = np.transpose(self.F)

        ## Initialize the measurements matrix
        # tracks 6 things: (x,y,yaw,yaw_rate,v_l,v_r), where x/y are already converted from GPS
        self.Z = np.transpose(np.matrix([0,0,0,0,0,0]))
        # buffer to hold measurements as they come in (outside the measure stage)
        self.Z_buffer = [0,0,0,0,0,0] 

        ## Initialize the observation matrix
        self.H = np.matrix([
            [1,0,0,0,0,0,0,0],
            [0,1,0,0,0,0,0,0],
            [0,0,0,0,1,0,0,0],
            [0,0,0,0,0,1,0,0],
            [0,0,0,0,0,0,1,0],
            [0,0,0,0,0,0,0,1]])
        self.H_trans = np.transpose(self.H)

        ## Initialize the covariance matrix with temporary values for the useful entries
        self.P = np.multiply(0.1,I_8)
        self.P_next = self.P
        print("initialized KF")

        ## Create necessary global variables
        self.start_gps = None

    def predict(self):
        ## Update the state transition matrix, F and F_trans
        cos_phi = cos(self.X[4])
        sin_phi = sin(self.X[4])
        self.F[1,6] = 0.5*WHEEL_RADIUS*cos_phi
        self.F[1,7] = 0.5*WHEEL_RADIUS*cos_phi
        self.F[3,6] = 0.5*WHEEL_RADIUS*sin_phi
        self.F[3,7] = 0.5*WHEEL_RADIUS*sin_phi
        # update transpose entries too
        self.F_trans[6,1] = 0.5*WHEEL_RADIUS*cos_phi
        self.F_trans[7,1] = 0.5*WHEEL_RADIUS*cos_phi
        self.F_trans[6,3] = 0.5*WHEEL_RADIUS*sin_phi
        self.F_trans[7,3] = 0.5*WHEEL_RADIUS*sin_phi

        ## Calculate predicted state for next iteration using dynamic model
        # state extrapolation: X(n+1) = F*X(n) + w (ignore process noise w)
        self.X_next = np.matmul(self.F,self.X)

        ## Extrapolate the estimate uncertainty
        # covariance extrapolation: P(n+1) = F*P(n)*F^T + Q
        self.P_next = np.matmul(np.matmul(self.F,self.P),self.F_trans) + self.Q

    def measure(self):
        ## Input measurement uncertainty
        # could update R, but we assume it's constant

        ## Load measured values from the buffer
        self.Z = np.transpose(np.matrix([self.Z_buffer[0],self.Z_buffer[1],self.Z_buffer[2],self.Z_buffer[3],self.Z_buffer[4],self.Z_buffer[5]]))

    def update(self):
        # NOTE We're interested to see if it would be better to update
        # P directly in predict() rather than holding the P_next buffer.
        # Uncomment this line to check an equivalent of this setup.
        self.P = self.P_next

        ## Calculate kalman gain
        # compute innovation covariance: S = H*P*H^T + R
        S = np.matmul(np.matmul(self.H,self.P),self.H_trans) + self.R #6x6 = 6x8 * 8x8 * 8x6 + 6x6
        # optimal kalman gain: K = P*H^T*S^-1
        S_inv = np.linalg.pinv(S) #6x6
        # NOTE not sure why but it blows up unless we multiply by H^T at the end of this
        #self.K = np.matmul(np.matmul(self.P,self.H_trans),S_inv) #8x6 = 8x8 * 8x6 * 6x6
        self.K = np.multiply(np.matmul(np.matmul(self.P,self.H_trans),S_inv), self.H_trans)
        print(self.K)

        ## Estimate the current state using the state update equation
        # state update: X(n+1) = X(n) + K*(Z-H*X(n))
        self.X = self.X_next + np.matmul(self.K,(self.Z-np.matmul(self.H,self.X_next))) #8x1 = 8x1 + 8x6 * (6x1 - 6x8 * 8x1)
        
        ## Update the current estimate uncertainty
        # covariance update: P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
        self.P = np.matmul(np.matmul((I_8-np.matmul(self.K,self.H)),self.P_next),np.transpose(I_8-np.matmul(self.K,self.H))) + np.matmul(np.matmul(self.K,self.R),np.transpose(self.K))
        # or P = (I-K*H)*P (simple version, unstable with confounding errors)
        #self.P = np.matmul((I_8-np.matmul(self.K,self.H)),self.P_next) #8x8 = (8x8 - 8x6 * 6x8) * 8x8

    def set_start_gps(self, gps_msg):
        if self.start_gps is None:
            self.start_gps = gps_msg
        else:
            # average the start position over time (before we start moving) to make it more accurate.
            l_rate = 0.1 # learning rate = weight given to new measurements
            self.start_gps.latitude = (1 - l_rate) * self.start_gps.latitude + l_rate * gps_msg.latitude
            self.start_gps.longitude = (1 - l_rate) * self.start_gps.longitude + l_rate * gps_msg.longitude

    def measure_gps(self, gps_msg):
        # we're treating this input as a direct measure of x and y.
        if gps_msg.hasSignal:
            self.Z_buffer[0] = (gps_msg.latitude - self.start_gps.latitude) * LAT_TO_M
            self.Z_buffer[1] = (gps_msg.longitude - self.start_gps.longitude) * LON_TO_M

    def measure_imu(self, imu_msg):
        # save robot's current heading in radians (0 north, CW).
        orientation = imu_msg.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        # do bad stuff to make the simulator work
        # quaternion = [-quaternion[3], quaternion[1], -quaternion[2], quaternion[0]]
        # shit_ass = [f"{x * 180 / 3.1415:4.01f}" for x in transformations.euler_from_quaternion(quaternion)]
        # print(shit_ass)
        yaw_rads = transformations.euler_from_quaternion(quaternion)[2]
        self.Z_buffer[2] = yaw_rads
        yaw_rate = imu_msg.angular_velocity.z
        self.Z_buffer[3] = yaw_rate

    def measure_velocities(self, vel_msg):
        # read the linear velocities from each wheel's encoder,
        # and convert to angular velocities for our measurement.
        self.Z_buffer[4] = vel_msg.leftVel #/ WHEEL_RADIUS
        self.Z_buffer[5] = vel_msg.rightVel #/ WHEEL_RADIUS

    def calc_equiv_gps(self,x:float,y:float):
        # calculate the equivalent GPS coords for our EKF position
        lat = x / LAT_TO_M + self.start_gps.latitude
        lon = y / LON_TO_M + self.start_gps.longitude
        return (lon,lat)

    def get_state_msg(self, mobi_start:bool):
        # create the state message that will be published.
        state_msg = EKFState()
        if mobi_start:
            cur_gps = self.calc_equiv_gps(self.X[0], self.X[2])
            state_msg.latitude = cur_gps[0]
            state_msg.longitude = cur_gps[1]
        state_msg.x = self.X[0]
        state_msg.x_velocity = self.X[1]
        state_msg.y = self.X[2]
        state_msg.y_velocity = self.X[3]
        state_msg.yaw = self.X[4]
        state_msg.yaw_rate = self.X[5]
        state_msg.left_velocity = self.X[6] * WHEEL_RADIUS
        state_msg.right_velocity = self.X[7] * WHEEL_RADIUS
        return state_msg

# function to generate the jacobian for our setup and print it.
def easy_print_jacobian():
    x_dot = Symbol("\dot{x}")
    y_dot = Symbol("\dot{y}")
    phi_dot = Symbol("\dot{phi}")
    wheel_rad = Symbol("R")
    wheelbase_len = Symbol("L")

    # State variables
    state_vars = ["x","x_dot","y","y_dot","phi","phi_dot","l","r"] #l=v_l, r=v_r

    # Differential drive approximation
    # Velocity calculations
    v_k = 0.5 * wheel_rad * (l + r)
    x_dot_k = v_k * cos(phi)
    y_dot_k = v_k * sin(phi)
    phi_dot_k = (wheel_rad / wheelbase_len) * (l - r)
    # Position calculations
    x_k = x + x_dot * t
    y_k = y + y_dot * t
    phi_k = phi + phi_dot * t

    # Setup vector of the functions
    f = Matrix([
        x_k,
        x_dot_k,
        y_k,
        y_dot_k,
        phi_k,
        phi_dot_k,
        l,
        r
        ])

    # Setup vector of the variables
    X = Matrix([x,x_dot,y,y_dot,phi,phi_dot,l,r])

    # IGVC Fk matrix
    model = f
    jac_model = f.jacobian(X)

    # Print the Jacobian to the console
    pprint(jac_model)
