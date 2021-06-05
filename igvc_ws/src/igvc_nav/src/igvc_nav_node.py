#!/usr/bin/env python3
import rospy
import math
import tf
from geometry_msgs.msg import Pose
from pure_pursuit import PurePursuit
from nav_msgs.msg import Path, Odometry
from igvc_msgs.msg import motors, EKFState, gps
from utilities.pp_viwer import setup_pyplot, draw_pp

SHOW_PLOTS = False
USE_SIM_TRUTH = False

pos = None
heading = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)

pp = PurePursuit()

# Coords of Oakland University: 42.6679° N, 83.2082° W
LAT_TO_M = 111086.33
LON_TO_M = 81972.46
# No Man's Land gps coords turned to meters relative to start. (x,y)
nml_start = (None, None)
nml_ramp = (None, None)
nml_end = (None, None)
# measured GPS coordinates on the course
meas_gps = (-42.66809, -83.21637, -42.66814, -83.21637, -42.66832, -83.21638)

def set_start_gps(start_gps):
    global nml_start, nml_ramp, nml_end
    # receive starting GPS position estimated by the EKF node,
    # and set position of known waypoints in meters relative to start.
    nml_start = ((meas_gps[0] - start_gps.latitude) * LAT_TO_M, (meas_gps[1] - start_gps.longitude) * LON_TO_M)
    nml_ramp = ((meas_gps[2] - start_gps.latitude) * LAT_TO_M, (meas_gps[3] - start_gps.longitude) * LON_TO_M)
    nml_end = ((meas_gps[4] - start_gps.latitude) * LAT_TO_M, (meas_gps[5] - start_gps.longitude) * LON_TO_M)

def ekf_update(ekf_state):
    global pos, heading

    pos = (ekf_state.x, ekf_state.y)
    heading = math.degrees(ekf_state.yaw)
    if heading < 0:
        heading += 360
    heading = 360 - heading

def true_pose_callback(data):
    global pos, heading

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    
    # if pitch > 0 and yaw > 0:
    #     yaw = math.pi - yaw
    # if pitch > 0 and yaw < 0:
    #     yaw = -math.pi - yaw

    pos = (data.position.x, data.position.y)
    heading = math.degrees(roll)

    if heading < 0:
        heading += 360

def global_path_update(data):
    points = [x.pose.position for x in data.poses] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

def get_angle_diff(angle1, angle2):
    delta = angle1 - angle2
    delta = (delta + 180) % 360 - 180
    return delta

def check_in_nml(cur_pos):
    # we're in NML if lat is between start and end (which are in a line),
    # and lon is within 5-10 meters of this line.
    return cur_pos[0] < nml_start[0] and cur_pos[0] > nml_end[0] and abs(cur_pos[1] - nml_start[1]) < 10

def timer_callback(event):
    if pos is None or heading is None:
        return

    cur_pos = (pos[0], pos[1])

    # check if we're in No Man's Land
    if check_in_nml(cur_pos):
        # do raw PID to heading to NML end location TODO
        heading_to_end = math.degrees(math.atan2(nml_end[1] - cur_pos[1], nml_end[0] - cur_pos[0]))
        # this assumes that reactive obstacle avoidance is happening on the control level,
        # so this commanded heading is the general path to follow, but will be ignored for 
        # small local things like avoiding the obstacles.
        # TODO make the obstacle avoidance w/ lidar data, perhaps in a different node.
    else:
        # we're on the regular course. do pure pursuit on local path as before.

        lookahead = None
        radius = 0.5 # Starting radius

        while lookahead is None and radius <= 3: # Look until we hit 3 meters max
            lookahead = pp.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            radius *= 1.2

        if SHOW_PLOTS:
            draw_pp(cur_pos, lookahead, pp.path)

        if lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1:
            # Get heading to the lookahead from current position
            heading_to_lookahead = math.degrees(math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0]))
            if heading_to_lookahead < 0:
                heading_to_lookahead += 360

            # Get difference in our heading vs heading to lookahead
            # Normalize error to -1 to 1 scale
            error = -get_angle_diff(heading, heading_to_lookahead)/180
            # print(f"am at {heading}, want to go to {heading_to_lookahead}")
            # print(f"error is {error}")

            # Base forward velocity for both wheels
            forward_speed = 0.6 * (1 - abs(error))**5
            # Define wheel linear velocities
            # Add proprtional error for turning.
            motor_pkt = motors()
            motor_pkt.left = (forward_speed - 0.4 * error)
            motor_pkt.right = (forward_speed + 0.4 * error)
            publy.publish(motor_pkt)
        else:
            # We couldn't find a suitable direction to head, so stop the robot.
            motor_pkt = motors()
            motor_pkt.left = 0
            motor_pkt.right = 0
            publy.publish(motor_pkt)
            return


def nav():
    rospy.init_node('nav_node', anonymous=True)

    if USE_SIM_TRUTH:
        rospy.Subscriber("/sim/true_pose", Pose, true_pose_callback)
    else:
        rospy.Subscriber("/igvc/state", EKFState, ekf_update)

    rospy.Subscriber("/igvc/global_path", Path, global_path_update)
    ## Subscribe to the start GPS position obtained by the EKF
    rospy.Subscriber("/igvc/start_gps", gps, set_start_gps, queue_size=1)

    rospy.Timer(rospy.Duration(0.05), timer_callback)

    if SHOW_PLOTS:
        setup_pyplot()

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

