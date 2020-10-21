#!/usr/bin/env python
import rospy
import math
import tf
from pure_pursuit import PurePursuit
from nav_msgs.msg import Path, Odometry
from igvc_msgs.msg import motors, EKFState
from utilities.pp_viwer import setup_pyplot, draw_pp

SHOW_PLOTS = False
WHEEL_RADIUS = 0.127

pos = None
heading = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)

pp = PurePursuit()

def ekf_update(ekf_state):
    global pos, heading

    pos = (ekf_state.x, ekf_state.y)
    heading = math.degrees(ekf_state.global_heading)

def global_path_update(data):
    points = [x.pose.position for x in data.poses] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

def timer_callback(event):
    if pos is None or heading is None:
        return

    cur_pos = (pos[0], pos[1])

    lookahead = None
    radius = 0.3 # Start with a radius of 0.3 meters

    while lookahead is None and radius <= 3: # Look until we hit 3 meters max
        lookahead = pp.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
        radius *= 1.2

    if SHOW_PLOTS:
        draw_pp(cur_pos, lookahead, pp.path)

    if lookahead is not None:
        # Get heading to to lookahead from current position
        heading_to_lookahead = math.degrees(math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0]))
        if heading_to_lookahead < 0:
            heading_to_lookahead += 360

        # Get difference in our heading vs heading to lookahead
        heading_error = heading_to_lookahead - heading
        heading_error = (heading_error + 180) % 360 - 180

        # Normalize error to -1 to 1 scale
        error = heading_error/180

        # Base forward velocity for both wheels
        forward_speed = 0.8 * (1 - abs(error))**5

        # Define wheel linear velocities
        # Add proprtional error for turning.
        # TODO: PID instead of just P
        motor_pkt = motors()
        motor_pkt.left = (forward_speed - 0.4 * error) / WHEEL_RADIUS
        motor_pkt.right = (forward_speed + 0.4 * error) / WHEEL_RADIUS

        publy.publish(motor_pkt)


def nav():
    rospy.init_node('nav_node', anonymous=True)

    rospy.Subscriber("/igvc_ekf/filter_output", EKFState, ekf_update)
    rospy.Subscriber("/igvc/global_path", Path, global_path_update)

    rospy.Timer(rospy.Duration(0.05), timer_callback)

    if SHOW_PLOTS:
        setup_pyplot()

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

