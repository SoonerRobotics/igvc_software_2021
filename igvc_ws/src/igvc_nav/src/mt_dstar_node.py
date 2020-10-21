#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from igvc_msgs.msg import motors, EKFState
from igvc_msgs.srv import EKFService
import copy
import numpy as np
from path_planner.mt_dstar_lite import mt_dstar_lite
from utilities.dstar_viewer import draw_dstar, setup_pyplot

SHOW_PLOTS = False

motor_pub = rospy.Publisher("/igvc/motors_raw", motors, queue_size=10)
global_path_pub = rospy.Publisher("/igvc/global_path", Path, queue_size=1)

# Moving Target D* Lite
map_init = False
path_failed = False
planner = mt_dstar_lite()

# Location when map was made
map_reference = (0, 0, 0)

# Localization tracking
prev_state = (0, 0)  # x, y
GRID_SIZE = 0.1      # Map block size in meters

# Path tracking
path_seq = 0

# Cost map
cost_map = None

# Latest EKF update
curEKF = EKFState()

# Best position to head to on the map (D* goal pos)
best_pos = (0,0)

def ekf_callback(data):
    global curEKF
    curEKF = data

def c_space_callback(c_space):
    global cost_map, map_reference, map_init, best_pos

    if curEKF is None:
        return

    grid_data = c_space.data

    # Make a costmap
    temp_cost_map = [0] * 200 * 200

    # Find the best position
    temp_best_pos = (100, 100)
    best_pos_cost = -1000

    # Only look forward for the goal
    for row in range(200):
        for col in range(200):
            if row <= 101:
                temp_cost_map[(row * 200) + col] = grid_data[(row * 200) + col]
            else:
                temp_cost_map[(row * 200) + col] = 100

    # Breath-first look for good points
    # This allows us to find a point within the range of obstacles by not
    # exploring over obstacles.
    frontier = set()
    frontier.add((100,100))
    explored = set()

    depth = 0
    while depth < 50 and len(frontier) > 0:
        curfrontier = copy.copy(frontier)
        for pos in curfrontier:
            cost = -abs(pos[1] - 100) + depth - temp_cost_map[pos[0] * 200 + pos[1]]
            if cost > best_pos_cost:
                best_pos_cost = cost
                temp_best_pos = pos

            frontier.remove(pos)
            explored.add(pos[0] * 200 + pos[1])

            if pos[0] < 199 and temp_cost_map[(pos[0] + 1) * 200 + pos[1]] != 100 and (pos[0] + 1) * 200 + pos[1] not in explored:
                frontier.add((pos[0] + 1, pos[1]))
            if pos[0] > 0 and temp_cost_map[(pos[0] - 1) * 200 + pos[1]] != 100 and (pos[0] - 1) * 200 + pos[1] not in explored:
                frontier.add((pos[0] - 1, pos[1]))
            if pos[1] < 199 and temp_cost_map[pos[0] * 200 + pos[1] + 1] != 100 and pos[0] * 200 + pos[1] + 1 not in explored:
                frontier.add((pos[0], pos[1] + 1))
            if pos[1] > 0 and temp_cost_map[pos[0] * 200 + pos[1] - 1] != 100 and pos[0] * 200 + pos[1] - 1 not in explored:
                frontier.add((pos[0], pos[1] - 1))

        depth += 1

    map_reference = (curEKF.x, curEKF.y, curEKF.yaw)
    cost_map = temp_cost_map
    best_pos = temp_best_pos
    map_init = False

def path_point_to_global_pose_stamped(robot_pos, pp0, pp1):
    # Local path
    x = (robot_pos[0] - pp0) * GRID_SIZE
    y = (robot_pos[1] - pp1) * GRID_SIZE

    # Translate to global path
    dx = map_reference[0]
    dy = map_reference[1]
    psi = map_reference[2]

    new_x = x * math.cos(psi) - y * math.sin(psi) + dx
    new_y = y * math.cos(psi) + x * math.sin(psi) + dy

    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()

    point = Point()
    point.x = new_x
    point.y = new_y
    pose_stamped.pose.position = point

    return pose_stamped

def make_map(c_space):
    global planner, map_init, path_failed, prev_state, path_seq

    if cost_map is None or curEKF is None:
        return

    # Reset the path
    path = None

    robot_pos = (100 - int((curEKF.x - map_reference[0]) / GRID_SIZE), 100 - int((curEKF.y - map_reference[1]) / GRID_SIZE))

    # MOVING TARGET D*LITE
    # If this is the first time receiving a map, or if the path failed to be made last time (for robustness),
    # initialize the path planner and plan the first path

    # TODO: Make this not True again lol
    if True:
        planner.initialize(200, 200, robot_pos, best_pos, cost_map)
        path = planner.plan()
        map_init = True
    # Otherwise, replan the path
    else:
        # Transform the map to account for heading changes
        hdg = curEKF.yaw
        #TODO: rotate map to 0 degree heading

        # Calculate the map shift based on the change in EKF state
        map_shift = (int(curEKF.x / GRID_SIZE) - prev_state[0], int(curEKF.y / GRID_SIZE) - prev_state[1])
        prev_state = (int(curEKF.x / GRID_SIZE), int(curEKF.y / GRID_SIZE))

        # Request the planner replan the path
        path = planner.replan(robot_pos, best_pos, cost_map) #, map_shift) # TODO: add in shifting

    if path is not None:
        global_path = Path()

        header = Header()
        header.seq = path_seq
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        path_seq += 1

        global_path.header = header
        global_path.poses = [path_point_to_global_pose_stamped(robot_pos, path_point[0], path_point[1]) for path_point in path]
        global_path.poses.reverse() # reverse path becuz its backwards lol

        global_path_pub.publish(global_path)

    else:
        # Set the path failed flag so we can fully replan
        path_failed = True

        # path_msg = copy.deepcopy(c_space)
        # data = planner.get_search_space_map()
        # data[(best_pos[0]*200) + best_pos[1]] = 100
        # print(str(c_space.info.width) + " x " + str(c_space.info.height))
        # path_msg.data = data
        # path_pub.publish(path_msg)

    if SHOW_PLOTS:
        draw_dstar(robot_pos, best_pos, cost_map, path, fig_num=2)


def mt_dstar_node():
    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    rospy.Subscriber("/igvc_ekf/filter_output", EKFState, ekf_callback)

    # Make a timer to publish new paths
    timer = rospy.Timer(rospy.Duration(secs=0.5), make_map, oneshot=False)

    if SHOW_PLOTS:
        setup_pyplot()

    # Wait for topic updates
    rospy.spin()



# Main setup
if __name__ == '__main__':
    try:
        mt_dstar_node()
    except rospy.ROSInterruptException:
        pass
