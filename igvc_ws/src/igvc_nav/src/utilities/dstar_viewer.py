import matplotlib as mpl
import numpy as np
from matplotlib import pyplot as plt

def setup_pyplot():
    plt.ion()
    plt.show()

# Draws dstar path planning information to pyplot
def draw_dstar(start_pos, goal_pos, cost_map, path, fig_num=1):
    plt.figure(fig_num)
    plt.clf()
    
    if cost_map:
        plt.imshow(np.reshape(cost_map, (200, 200)), interpolation = 'nearest')

    if path:
        for point in path:
            plt.plot(point[1], point[0], '.', markersize=8, color="red")

    if start_pos:
        plt.plot(start_pos[1], start_pos[0], '.', markersize=8, color="black")
    
    if goal_pos:
        plt.plot(goal_pos[1],goal_pos[0], '.', markersize=8, color="pink")

    plt.draw()
    plt.pause(0.00000000001)