import numpy as np

from math import radians, degrees
import time

from sympy import sin, cos, asin, atan2, sqrt, Matrix, Symbol, eye
from sympy.abc import theta, phi, lamda, psi, p, x, y, v, a, t, r, l
from sympy import pprint

# Constants
L = 0.84455         # Wheelbase length (meters)
wheel_rad = 0.127   # Wheel radius (meters)
R = 6378137         # Earth radius (meters)

x = Symbol("x")

# State variables
state_vars = ["phi", "lamda", "theta", "x", "y", "psi", "v",  "a"]
ctrl_vars = ["l", "r"]

# Differential drive approximation
# Velocity calculations
v_k = 0.5 * wheel_rad * (l + r)
x_dot = v_k * cos(psi)
y_dot = v_k * sin(psi)
psi_dot = (wheel_rad / L) * (l - r)

# Position calculations
x_k = x + x_dot * t
y_k = y + y_dot * t
psi_k = psi + psi_dot * t
theta_k = theta + psi_dot * t

# GPS calcs
lat = phi + (v * t) * cos(theta) / R
lon = lamda + (v * t) * sin(theta) / (R * cos(phi))

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
model = f
jac_model = f.jacobian(X)

pprint(jac_model)