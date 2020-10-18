import numpy as np

from rrt.rrt_star import RRTStar
from search_space.search_space import SearchSpace
from utilities.obstacle_generation import roadmap1
from utilities.plotting import Plot

X_dimensions = np.array([(0, 2000), (0, 2000)])  # dimensions of Search Space
x_init = (1000, 100)  # starting location
x_goal = (1000, 1500)  # goal location

Q = np.array([(80, 40)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
rewire_count = 32  # optional, number of nearby branches to rewire
prc = 0.1  # probability of checking for a connection to goal

# create Search Space
Obstacles = roadmap1()
X = SearchSpace(X_dimensions, Obstacles)
n = 50

# create rrt_search
rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
path = rrt.rrt_star()

# plot
plot = Plot("rrt_star_2d_with_random_obstacles")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)