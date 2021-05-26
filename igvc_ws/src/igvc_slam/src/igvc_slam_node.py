#!/usr/bin/env python3

import copy
import numpy as np
import rospy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

# Configuration
wait_for_vision = True

# Publishers
config_pub = rospy.Publisher("/igvc_slam/local_config_space", OccupancyGrid, queue_size=1)

# Configuration space map
metadata = MapMetaData()
lidar_config_data = [0] * (200 * 200)
lanes_camera_config_data = [0] * (200 * 200)
lidar_hidden_layer = [0] * (200 * 200)
header = Header()
header.frame_id = "map"

# Initializiation
lidar_init = False
last_vision = None

def lidar_callback(data):
    # use the global vars
    global lidar_init, lidar_config_data, lidar_hidden_layer, metadata

    # Reset the hidden layer
    lidar_hidden_layer = [0] * (200 * 200)

    if last_vision is not None:
        cap_last_vision = last_vision
        pog = [x+y for x,y in zip(data.data, cap_last_vision)]
        data.data = pog
    elif wait_for_vision:
        return

    # HACK: eventually set this to be based on the map size and stuff
    metadata = MapMetaData(map_load_time = data.info.map_load_time, resolution=data.info.resolution,
                            width = data.info.width, height = data.info.height, origin = data.info.origin)
    lidar_init = True

    # Update the hidden layer before applying the new map to the current configuration space
    for x in range(200):
        for y in range(200):
            if data.data[x + y * 200] > 0:
                for x_i in range(-8,8):
                    for y_i in range(-8,8):
                        dist = (x_i)**2 + (y_i)**2
                        index = ((x + x_i)) + 200 * (y + y_i)

                        if 0 <= (x + x_i) < 200 and 0 <= (y + y_i) < 200 and dist <= 9 and lidar_hidden_layer[index] <= 100:
                            # obstacle expansion
                            lidar_hidden_layer[index] = 100
                        elif 0 <= (x + x_i) < 200 and 0 <= (y + y_i) < 200 and dist <= 64 and lidar_hidden_layer[index] <= dist * (-100/55) + (1280/11):
                            # linearly decay
                            lidar_hidden_layer[index] = int(dist * (-100/55) + (1280/11))

    # After updating the hidden layer, swap the hidden layer to the foreground to apply the configuration space
    tmp_cfg_space = lidar_config_data
    lidar_config_data = lidar_hidden_layer
    lidar_hidden_layer = tmp_cfg_space


def lanes_camera_callback(data):
    global last_vision
    last_vision = data.data

# TODO: currently this whole thing is set to use the most recent map frames from each perception unit (which is fine for now). In the future the sizing will change as the map grows.
def config_space_callback(event):
    # Use the global data
    global lidar_init, lidar_config_data, metadata

    if lidar_init is True:
        # TODO: Make this add the lidar config space to the camera config space
        config_space = lidar_config_data

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
    timer = rospy.Timer(rospy.Duration(secs=0.4), config_space_callback, oneshot=False)

    # Wait for topic updates
    rospy.spin()


# Main setup
if __name__ == '__main__':
    try:
        igvc_slam_node()
    except rospy.ROSInterruptException:
        pass