#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from igvc_msgs.msg import imuodom
from sensor_msgs.msg import Imu

publisher = rospy.Publisher("/igvc/imu", imuodom, queue_size=1)

class Euler:
    def __init__(self, yaw=0, pitch=0, roll=0):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

# from wikipedia "Conversion between quaternions and Euler angles"
def to_euler(w, x, y, z):
    angles = Euler()

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    angles.roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        angles.pitch = math.copysign(math.pi / 2, sinp)
    else:
        angles.pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    angles.yaw = math.atan2(siny_cosp, cosy_cosp)

    return angles

def imu_callback(data):
    imu_out = imuodom()

    imu_out.acceleration = data.linear_acceleration.y
    imu_out.heading = to_euler(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z).yaw

    publisher.publish(imu_out)

def imu_node():

    # Setup node
    rospy.init_node("imu_node", anonymous = True)

    # Subscribe to necessary topics
    rospy.Subscriber("/imu", Imu, imu_callback)

    # Wait for topic updates
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        pass
