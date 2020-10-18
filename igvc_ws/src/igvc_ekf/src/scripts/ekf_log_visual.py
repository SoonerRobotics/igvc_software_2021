from matplotlib import pyplot as plt
from numpy import genfromtxt

vel_data = genfromtxt('vel_log.csv', delimiter=',')
accel_data = genfromtxt('accel_log.csv', delimiter=',')