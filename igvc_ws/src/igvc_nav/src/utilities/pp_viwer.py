import matplotlib as mpl
from matplotlib import pyplot as plt

def setup_pyplot():
    plt.ion()
    plt.show()

# Draws pure pursuit information to pyplot
def draw_pp(agent_pos, lookahead_pos, waypoints, xlims=[-10,10], ylims=[-10,10], fig_num=1):
    plt.figure(fig_num)
    plt.clf()
    plt.xlim(xlims)
    plt.ylim(ylims)

    if waypoints:
        for point in waypoints:
            plt.plot(point[0], point[1], '.', markersize=6)

    if agent_pos:
        plt.plot(agent_pos[0], agent_pos[1], 's', markersize=8)

    if lookahead_pos:
        plt.plot(lookahead_pos[0], lookahead_pos[1], 'x', markersize=8)

    plt.draw()
    plt.pause(0.00000000001)