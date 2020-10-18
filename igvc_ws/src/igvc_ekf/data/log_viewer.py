#!/usr/bin/env python

import pandas as pd
from matplotlib import pyplot as plt

# Read the data files
ekf_file = pd.read_csv("export.csv", names=['id', 'lat', 'lon', 'hdg', 'ekf_x', 'ekf_y', 'yaw', 'nan'])
ground_truth_file = pd.read_csv("ground_truth.csv", names=['true_x', 'true_y'])

# Set the x,y coordinates
ekf_x = ekf_file['ekf_x']
ekf_y = ekf_file['ekf_y']
ground_truth_x = ground_truth_file['true_x']
ground_truth_y = ground_truth_file['true_y']

# Plot the data
fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.scatter(ekf_x, ekf_y, s=10, c='b', marker="s", label='EKF')
ax1.scatter(ground_truth_x, ground_truth_y, s=10, c='r', marker="o", label='ground_truth')
plt.legend(loc='upper right');
plt.show()
