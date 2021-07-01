#!/usr/bin/env python3
import rospy
import math
import tf
from geometry_msgs.msg import Pose
from pure_pursuit import PurePursuit
from nav_msgs.msg import Path, Odometry
from igvc_msgs.msg import motors, EKFState, velocity
from utilities.pp_viwer import setup_pyplot, draw_pp

SHOW_PLOTS = False
USE_SIM_TRUTH = False

pos = None
heading = None
ekf = None
mspeed = None
publy = rospy.Publisher('/igvc/motors_raw', motors, queue_size=1)

pp = PurePursuit()

def ekf_update(ekf_state):
    global pos, heading, ekf

    ekf = ekf_state

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

def velocity_callback(data):
    global mspeed
    mspeed = (data.leftVel, data.rightVel)

def global_path_update(data):
    points = [x.pose.position for x in data.poses] # Get points from Path
    pp.set_points([(_point.x, _point.y) for _point in points]) # Give PurePursuit the points

def get_angle_diff(angle1, angle2):
    delta = angle1 - angle2
    delta = (delta + 180) % 360 - 180
    return delta

def clamp(val, min, max):
    if val < min:
        return min
    if val > max:
        return max
    return val

def timer_callback(event):
    if pos is None or heading is None:
        return

    cur_pos = (pos[0], pos[1])

    lookahead = None
    radius = 0.8 # Starting radius

    while lookahead is None and radius <= 3: # Look until we hit 3 meters max
        lookahead = pp.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
        radius *= 1.2

    if SHOW_PLOTS:
        draw_pp(cur_pos, lookahead, pp.path)

    if lookahead is not None and mspeed is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1:
        # Get heading to to lookahead from current position
        heading_to_lookahead = math.degrees(math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0]))
        if heading_to_lookahead < 0:
            heading_to_lookahead += 360

        # Get difference in our heading vs heading to lookahead
        # Normalize error to -1 to 1 scale
        error = get_angle_diff(heading, heading_to_lookahead)/180

        # print(f"am at {cur_pos[0]:0.02f},{cur_pos[1]:0.02f}, want to go to {lookahead[0]:0.02f},{lookahead[1]:0.02f}")
        # print(f"angle delta: {error * 180:0.01f}")

        # print(f"error is {error}")
        if abs(error) < 2.0/180:
            error = 0

        # Base forward velocity for both wheels
        forward_speed = 0.65 * (1 - abs(error))**3

        # Define wheel linear velocities
        # Add proprtional error for turning.
        # TODO: PID instead of just P
        motor_pkt = motors()
        motor_pkt.left = (forward_speed - clamp(0.3 * error, -0.2, 0.2))
        motor_pkt.right = (forward_speed + clamp(0.3 * error, -0.2, 0.2))

        if motor_pkt.left < mspeed[0] - 0.1:
            motor_pkt.left = mspeed[0] - 0.1

        if motor_pkt.left > mspeed[0] + 0.1:
            motor_pkt.left = mspeed[0] + 0.1

        if motor_pkt.right < mspeed[1] - 0.1:
            motor_pkt.right = mspeed[1] - 0.1

        if motor_pkt.right > mspeed[1] + 0.1:
            motor_pkt.right = mspeed[1] + 0.1

        publy.publish(motor_pkt)
    else:
        # We couldn't find a suitable direction to head, stop the robot.
        motor_pkt = motors()
        motor_pkt.left = -0.25
        motor_pkt.right = -0.25

        publy.publish(motor_pkt)


def nav():
    rospy.init_node('nav_node', anonymous=True)

    if USE_SIM_TRUTH:
        rospy.Subscriber("/sim/true_pose", Pose, true_pose_callback)
    else:
        rospy.Subscriber("/igvc/state", EKFState, ekf_update)

    rospy.Subscriber("/igvc/global_path", Path, global_path_update)
    rospy.Subscriber("/igvc/velocity", velocity, velocity_callback)

    rospy.Timer(rospy.Duration(0.05), timer_callback)

    if SHOW_PLOTS:
        setup_pyplot()

    rospy.spin()

if __name__ == '__main__':
    try:
        nav()
    except rospy.ROSInterruptException:
        pass

