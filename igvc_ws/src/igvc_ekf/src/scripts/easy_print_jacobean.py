# We just need this to calculate the Jacobean for us
# so we can manually encode it into the equations in ekf.cpp

from sympy import sin, cos, Matrix, Symbol, eye
from sympy.abc import phi, x, y, t, r, l
from sympy import pprint


# Constants
wheelbase_len = 0.84455 # Wheelbase length (meters)
wheel_rad = 0.127 # Wheel radius (meters)
#R = 6378137 # Earth radius (meters)

x_dot = Symbol("\dot{x}")
y_dot = Symbol("\dot{y}")
phi_dot = Symbol("\dot{phi}")

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

# # Input dictionary setup
# input_dict = {self.state_vars[i]: last_xk[i] for i in range(0, len(self.state_vars))}
# input_dict['t'] = dt
# ctrl_dict = {self.ctrl_vars[i]: ctrl[i] for i in range(0, len(self.ctrl_vars))}

# F = self.jac_model.subs(input_dict).subs(ctrl_dict)
# new_state = self.model.subs(input_dict).subs(ctrl_dict)
# P = (F * P * F.T) + self.Q

# Print the Jacobean to the console
pprint(jac_model)
