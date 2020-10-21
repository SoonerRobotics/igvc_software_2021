"""
Motion model code and simulation for kalman filter model
"""
from math import radians
import time

from matplotlib import pyplot as plt

from ekf import EKF

# Main program
if __name__ == "__main__":

    # GPS and local heading
    hdg = radians(0)
    psi_k = radians(0)

    # Driving simulation
    last_xk = [
            radians(35.210467),
            radians(-97.441811),
            hdg,
            0,
            0,
            psi_k,
            0,
            0,
            0,
            0
        ]
    
    # Commanded wheel velocities
    ctrl_signal = [
        3.14,
        6.3
    ]

    # Init the ekf
    ekf = EKF(last_xk)


    print(ekf.motion.get_jac())
    exit(0)

    dt = 1.0 / 20.0
    sim_time = 4
    sim_steps = int(round(sim_time / dt))

    points = []

    # Simulate the EKF
    start_time = time.time()
    for _ in range(0, sim_steps):
        # last_xk = motion.run_filter(last_xk, ctrl_signal, dt)
        last_xk = ekf.run_filter(last_xk, ctrl_signal, dt)
        points.append((last_xk[3], last_xk[4]))

    end_time = time.time()

    # Plot the EKF x,y position estimates
    plt.scatter(*zip(*points))
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Predicted Local Frame - EKF")
    plt.show()
    
    print("elapsed time:", end_time-start_time)
