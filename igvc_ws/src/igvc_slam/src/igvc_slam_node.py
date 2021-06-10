#!/usr/bin/env python3

import copy
import numpy as np
import rospy
import math
import itertools

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

# Configuration
wait_for_vision = True

# Publishers
config_pub = rospy.Publisher("/igvc_slam/local_config_space", OccupancyGrid, queue_size=1)

# Configuration space map
origin = Pose()
origin.position.x = -10
origin.position.y = -10
metadata = MapMetaData(map_load_time = rospy.Time(), resolution=0.1,
                            width = 200, height = 200, origin = origin)
header = Header()
header.frame_id = "map"

max_range = 8
xxxs = list(range(-max_range, max_range + 1))
circle_around_indicies = []
for x in xxxs:
    for y in xxxs:
        if math.sqrt(x**2 + y**2) < max_range:
            circle_around_indicies.append((x, y, math.sqrt(x**2 + y**2)))
no_go_range = 4

# Initializiation
last_lidar = None
last_vision = None

def lidar_callback(data):
    # use the global vars
    global last_lidar
    last_lidar = data


def lanes_camera_callback(data):
    global last_vision
    last_vision = data.data

# TODO: currently this whole thing is set to use the most recent map frames from each perception unit (which is fine for now). In the future the sizing will change as the map grows.
def config_space_callback(event):

    if last_vision is None and last_lidar is None:
        return

    # Reset the hidden layer
    config_space = [0] * (200 * 200)

    combined_maps = None
    if last_vision is not None and last_lidar is not None:
        combined_maps = [x+y for x,y in zip(last_lidar.data, last_vision)]
    elif last_vision is not None:
        combined_maps = last_vision
    else:
        combined_maps = last_lidar.data

    # Update the hidden layer before applying the new map to the current configuration space
    for x in range(200):
        for y in range(200):
            if combined_maps[x + y * 200] > 0:
                for x_i, y_i, dist in circle_around_indicies:
                        index = (x + x_i) + 200 * (y + y_i)

                        if 0 <= (x + x_i) < 200 and 0 <= (y + y_i) < 200:
                            val_at_index = config_space[index]
                            linear_t = max_range**2 - ((dist - no_go_range) / (max_range - no_go_range) * max_range**2)

                            if dist <= no_go_range:
                                # obstacle expansion
                                config_space[index] = 100
                            elif dist <= 100 and val_at_index <= linear_t:
                                # linearly decay
                                config_space[index] = int(linear_t)

    # Publish the configuration space
    header.stamp = rospy.Time.now()
    config_msg = OccupancyGrid(header=header, info=metadata, data=config_space)
    config_pub.publish(config_msg)



def igvc_slam_node():
    # Set up node
    rospy.init_node("igvc_slam_node")

    # Subscribe to necessary topics
    map_sub = rospy.Subscriber("/igvc_vision/map", OccupancyGrid, lidar_callback, queue_size=10)
    vision_sub = rospy.Subscriber("/igvc/lane_map", OccupancyGrid, lanes_camera_callback, queue_size=10)

    # Make a timer to publish configuration spaces periodically
    timer = rospy.Timer(rospy.Duration(secs=0.2), config_space_callback, oneshot=False)

    # Wait for topic updates
    rospy.spin()


# Main setup
if __name__ == '__main__':
    try:
        igvc_slam_node()
    except rospy.ROSInterruptException:
        pass
