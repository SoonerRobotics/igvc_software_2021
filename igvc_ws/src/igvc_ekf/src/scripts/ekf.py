import numpy as np

from math import radians, degrees
import time

from sympy import sin, cos, asin, atan2, sqrt, Matrix, Symbol, eye
from sympy.abc import theta, phi, lamda, psi, p, x, y, v, a, t, r, l

class EKF:

    def __init__(self, x_o):
        # Initialize state and covariance
        self.x_k = x_o
        self.P_k = eye(11) * 0.8

        # Initialize motion model
        self.motion = MotionModel(self.P_k)

        # Initialize measurement model
        self.measure = MeasurementModel()

    def run_filter(self, sensor_data, ctrl, dt):
        self.x_k, self.P_k = self.motion.run_model(self.x_k, self.P_k, ctrl, dt)

        return self.x_k



class MotionModel:
    """
    SCR IGVC Motion Model
    """

    # Constants
    L = 0.84455         # Wheelbase length (meters)
    wheel_rad = 0.127   # Wheel radius (meters)
    R = 6378137         # Earth radius (meters)

    def __init__(self, Po):
        x = Symbol("x")

        # State variables
        self.state_vars = ["phi", "lamda", "theta", "x", "y", "psi", "v",  "a"]
        self.ctrl_vars = ["l", "r"]

        # Differential drive approximation
        # Velocity calculations
        v_k = 0.5 * self.wheel_rad * (l + r)
        x_dot = v_k * cos(psi)
        y_dot = v_k * sin(psi)
        psi_dot = (self.wheel_rad / self.L) * (l - r)

        # Position calculations
        x_k = x + x_dot * t
        y_k = y + y_dot * t
        psi_k = psi + psi_dot * t
        theta_k = theta + psi_dot * t

        # GPS calcs
        lat = phi + (v * t) * cos(theta) / self.R
        lon = lamda + (v * t) * sin(theta) / (self.R * cos(phi))

        f = Matrix([
            lat,
            lon,
            theta_k,
            x_k,
            y_k,
            psi_k,
            v_k,
            psi_dot,
            l,
            r,
            a
            ])

        X = Matrix([phi, lamda, theta, x, y, psi, v, p, l, r, a])

        # IGVC Fk matrix
        self.model = f
        self.jac_model = f.jacobian(X)
        self.Q = eye(11) * 0.2

        self.csv_data = "lat, lon\n"

    def run_model(self, last_xk, P, ctrl, dt):
        last_xk_mat = Matrix(last_xk[:11])

        # Input dictionary setup
        input_dict = {self.state_vars[i]: last_xk[i] for i in range(0, len(self.state_vars))}
        input_dict['t'] = dt
        ctrl_dict = {self.ctrl_vars[i]: ctrl[i] for i in range(0, len(self.ctrl_vars))}

        F = self.jac_model.subs(input_dict).subs(ctrl_dict)
        new_state = self.model.subs(input_dict).subs(ctrl_dict)
        P = (F * P * F.T) + self.Q

        self.csv_data += str(degrees(new_state[0])) + "," + str(degrees(new_state[1])) + "\n"

        return (new_state, P)

    def get_model(self):
        return self.model

    def get_cov(self):
        return self.cov_matrix

    def get_jac(self):
        return self.jac_model

    def print_gps(self):
        print(self.gps_lat, ", ", self.gps_lon)

    def export_data(self):
        csv_file = open("export.csv", "w")
        csv_file.write(self.csv_data)
        csv_file.close()


class MeasurementModel:

    def __init__(self):
        pass