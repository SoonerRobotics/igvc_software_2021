#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from igvc_msgs.msg import motors, EKFState, gps
from igvc_msgs.srv import EKFService
import tf
import copy
import numpy as np
from path_planner.mt_dstar_lite import mt_dstar_lite
from utilities.dstar_viewer import draw_dstar, setup_pyplot

SHOW_PLOTS = False
USE_SIM_TRUTH = False

# Coords of Oakland University: 42.6679° N, 83.2082° W
LAT_TO_M = 111086.33
LON_TO_M = 81972.46
# No Man's Land gps coords turned to meters relative to start. (x,y)
nml_start = (None, None)
nml_ramp = (None, None)
nml_end = (None, None)
# measured GPS coordinates on the course
meas_gps = (-42.66809, -83.21637, -42.66814, -83.21637, -42.66832, -83.21638)
# scaling factor to emphasize the importance of making forward progress in NML
fwd_scale = 1

def set_start_gps(start_gps):
    global nml_start, nml_ramp, nml_end
    # receive starting GPS position estimated by the EKF node,
    # and set position of known waypoints in meters relative to start.
    nml_start = ((meas_gps[0] - start_gps.latitude) * LAT_TO_M, (meas_gps[1] - start_gps.longitude) * LON_TO_M)
    nml_ramp = ((meas_gps[2] - start_gps.latitude) * LAT_TO_M, (meas_gps[3] - start_gps.longitude) * LON_TO_M)
    nml_end = ((meas_gps[4] - start_gps.latitude) * LAT_TO_M, (meas_gps[5] - start_gps.longitude) * LON_TO_M)

def check_in_nml(curEKF):
    global fwd_scale
    # we're in NML if lat is between start and end (which are in a line),
    # and lon is within 5-10 meters of this line.
    if curEKF.x < nml_start[0] and curEKF.x > nml_end[0] and abs(curEKF.y - nml_start[1]) < 10:
        fwd_scale = 3
    else:
        fwd_scale = 1

global_path_pub = rospy.Publisher("/igvc/global_path", Path, queue_size=1)
local_path_pub = rospy.Publisher("/igvc/local_path", Path, queue_size=1)

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

# fitness map
fitness_map = None

# Latest EKF update
curEKF = EKFState()

# Best position to head to on the map (D* goal pos)
best_pos = (0,0)

def ekf_callback(data):
    global curEKF
    curEKF = data
    check_in_nml(curEKF)

def true_pose_callback(data):
    global curEKF
    curEKF = EKFState()
    curEKF.x = data.position.x
    curEKF.y = data.position.y
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    curEKF.yaw = roll

def c_space_callback(c_space):
    global fitness_map, map_reference, map_init, best_pos

    if curEKF is None:
        return

    grid_data = c_space.data

    # Make a cost map
    temp_cost_map = [0] * 200 * 200

    # Find the best position
    temp_best_pos = (100, 100)
    best_pos_fitness = -1000

    # Only look forward for the goal
    for x in range(200):
        for y in range(200):
            if x >= 100:
                temp_cost_map[(y * 200) + x] = grid_data[(y * 200) + x]
            else:
                temp_cost_map[(y * 200) + x] = 100

    # Breadth-first look for good points
    # This allows us to find a point within the range of obstacles by not
    # exploring over obstacles.
    frontier = set()
    frontier.add((100,100))
    explored = set()

    depth = 0
    while depth < 50 and len(frontier) > 0:
        curfrontier = copy.copy(frontier)
        for pos in curfrontier:
            # Fitness at a point is the sum of
            # - Shifted X value (encourage forward) (100 is highest possible X)
            # - Negative shifted Y value (discourage left/right)
            # - Depth (number of breadth-first search iterations)
            # - Config space fitness
            fitness = fwd_scale * (pos[0] - 100) + -abs(pos[1] - 100) + depth - temp_cost_map[pos[1] * 200 + pos[0]]
            if fitness > best_pos_fitness:
                best_pos_fitness = fitness
                temp_best_pos = pos

            frontier.remove(pos)
            explored.add(pos[1] * 200 + pos[0])

            # Look left/right for good points
            if pos[1] < 199 and temp_cost_map[(pos[1] + 1) * 200 + pos[0]] != 100 and (pos[1] + 1) * 200 + pos[0] not in explored:
                frontier.add((pos[0], pos[1]+1))
            if pos[1] > 0 and temp_cost_map[(pos[1] - 1) * 200 + pos[0]] != 100 and (pos[1] - 1) * 200 + pos[0] not in explored:
                frontier.add((pos[0], pos[1]-1))

            # Look forward/back for good points
            if pos[0] < 199 and temp_cost_map[pos[1] * 200 + pos[0] + 1] != 100 and pos[1] * 200 + pos[0] + 1 not in explored:
                frontier.add((pos[0]+1, pos[1]))
            if pos[0] > 0 and temp_cost_map[pos[1] * 200 + pos[0] - 1] != 100 and pos[1] * 200 + pos[0] - 1 not in explored:
                frontier.add((pos[0]-1, pos[1]))

        depth += 1

    map_reference = (curEKF.x, curEKF.y, curEKF.yaw)
    fitness_map = temp_cost_map
    best_pos = temp_best_pos
    map_init = False

def path_point_to_global_pose_stamped(robot_pos, pp0, pp1, header):
    # Local path
    x = (pp0 - robot_pos[0]) * GRID_SIZE
    y = (pp1 - robot_pos[1]) * GRID_SIZE

    # Translate to global path
    dx = map_reference[0]
    dy = map_reference[1]
    psi = map_reference[2]

    new_x = x * math.cos(psi) - y * math.sin(psi) + dx
    new_y = y * math.cos(psi) + x * math.sin(psi) + dy

    pose_stamped = PoseStamped(header=header)
    pose_stamped.pose = Pose()

    point = Point()
    point.x = new_x
    point.y = new_y
    pose_stamped.pose.position = point

    return pose_stamped

def path_point_to_local_pose_stamped(pp0, pp1, header):
    pose_stamped = PoseStamped(header=header)
    pose_stamped.pose = Pose()

    point = Point()
    point.x = (pp0 - 100) / 10
    point.y = (pp1 - 100) / 10
    pose_stamped.pose.position = point

    return pose_stamped

def make_map(c_space):
    global planner, map_init, path_failed, prev_state, path_seq

    if fitness_map is None or curEKF is None:
        return

    # Reset the path
    path = None

    robot_pos = (100, 100)

    # MOVING TARGET D*LITE
    # If this is the first time receiving a map, or if the path failed to be made last time (for robustness),
    # initialize the path planner and plan the first path

    # TODO: Make this not True again lol
    if True:
        planner.initialize(200, 200, robot_pos, best_pos, fitness_map)
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
        path = planner.replan(robot_pos, best_pos, fitness_map) #, map_shift) # TODO: add in shifting

    if path is not None:
        header = Header()
        header.seq = path_seq
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        path_seq += 1

        global_path = Path(header = header)
        global_path.poses = [path_point_to_global_pose_stamped(robot_pos, path_point[0], path_point[1], header) for path_point in path]
        global_path.poses.reverse() # reverse path becuz its backwards lol

        local_path = Path(header = header)
        local_path.poses = [path_point_to_local_pose_stamped(path_point[0], path_point[1], header) for path_point in path]
        local_path.poses.reverse() # reverse path becuz its backwards lol

        global_path_pub.publish(global_path)
        local_path_pub.publish(local_path)

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
        draw_dstar(robot_pos, best_pos, fitness_map, path, fig_num=2)


def mt_dstar_node():
    # Setup node
    rospy.init_node("mt_dstar_node")

    # Subscribe to necessary topics
    rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, c_space_callback, queue_size=1)  # Mapping
    if USE_SIM_TRUTH:
        rospy.Subscriber("/sim/true_pose", Pose, true_pose_callback)
    else:
        rospy.Subscriber("/igvc/state", EKFState, ekf_callback)
    
    ## Subscribe to the start GPS position obtained by the EKF
    rospy.Subscriber("/igvc/start_gps", gps, set_start_gps, queue_size=1)

    # Make a timer to publish new paths
    timer = rospy.Timer(rospy.Duration(secs=0.3), make_map, oneshot=False)

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
