#!/usr/bin/env python3

import rospy
import json
import threading
import serial
import can
import struct

from std_msgs.msg import String
from igvc_msgs.msg import motors, velocity, gps

# ROS node that facilitates all serial communications within the robot
# Subscribes to motor values
# Publishes GPS coordinates

serials = {}
cans = {}

MAX_SPEED = 2.2 # m/s
CAN_SEND_VELOCITY_ID = 10
CAN_RECV_VELOCITY_ID = 11

class VelocityCANReadThread(threading.Thread):
    def __init__(self, can_obj, topic):
        threading.Thread.__init__(self)

        self.can_obj = can_obj

        # Allow timeout of up to 1 second on reads. This could be set to None to have infinite timeout,
        # but that would hault the node when it tries to exit. Need to make sure the while loop condition is
        # semi-regularly checked. This is better than rospy.Rate because it will continously wait for new message
        # instead of only checking on a fixed interval.
        # self.can_obj.timeout = 1

        # Assumes String type for now. This class will need to be adapted in the future for different message types.
        self.publisher = rospy.Publisher(topic, velocity, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            msg = self.can_obj.recv(timeout=1)

            if msg and msg.arbitration_id == CAN_RECV_VELOCITY_ID:
                left_speed, right_speed, max_speed = struct.unpack("bbB", msg.data)

                velPkt = velocity()
                velPkt.leftVel = left_speed / 127 * max_speed / 10
                velPkt.rightVel = right_speed / 127 * max_speed / 10

                print(f"Received {velPkt.leftVel} {velPkt.rightVel}")

                self.publisher.publish(velPkt)


class GPSSerialReadThread(threading.Thread):
    def __init__(self, serial_obj, topic):
        threading.Thread.__init__(self)

        self.serial_obj = serial_obj

        # Allow timeout of up to 1 second on reads. This could be set to None to have infinite timeout,
        # but that would hault the node when it tries to exit. Need to make sure the while loop condition is
        # semi-regularly checked. This is better than rospy.Rate because it will continously wait for new message
        # instead of only checking on a fixed interval.
        self.serial_obj.timeout = 1

        # Assumes String type for now. This class will need to be adapted in the future for different message types.
        self.publisher = rospy.Publisher(topic, gps, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            coord = self.serial_obj.readline().decode()[:-1] # Assume all messages end in newline character. This is standard among SCR IGVC serial messages.

            try:
                if coord:
                    coord_json = json.loads(coord)

                    coord_msg = gps()
                    coord_msg.latitude = coord_json['latitude']
                    coord_msg.longitude = coord_json['longitude']
                    coord_msg.hasSignal = coord_json['hasSignal']

                    self.publisher.publish(coord_msg)
            except ValueError:
                pass

# Constructs motor message from given data and sends to serial
def motors_out(data):
    left_speed = int(data.left / MAX_SPEED * 127)
    right_speed = int(data.right / MAX_SPEED * 127)

    packed_data = struct.pack('bbB', left_speed, right_speed, int(MAX_SPEED * 10))

    can_msg = can.Message(arbitration_id=CAN_SEND_VELOCITY_ID, data=packed_data)

    cans["motor"].send(can_msg)

    print(f"Sent {data.left} {data.right}")


# Initialize the serial node
# Node handles all serial communication within the robot (motor, GPS)
def init_serial_node():
    
    # Setup serial node
    rospy.init_node("serial_node", anonymous = False)

    # Setup motor serial and subscriber
    # motor_serial = serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 115200)
    motor_can = cans["motor"] = can.ThreadSafeBus(bustype='slcan', channel='/dev/igvc-can-835', bitrate=100000)
    rospy.Subscriber('/igvc/motors_raw', motors, motors_out)
    
    motor_response_thread = VelocityCANReadThread(can_obj = cans["motor"], topic = '/igvc/velocity')
    motor_response_thread.start()

    # Setup GPS serial and publisher
    # gps_serial = serials["gps"] = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600)

    # gps_response_thread = GPSSerialReadThread(serial_obj = serials["gps"], topic = '/igvc/gps')
    # gps_response_thread.start()
    
    # Wait for topic updates
    rospy.spin()

    # Close the serial ports when program ends
    print("Closing threads")
    motor_can.close()
    #gps_serial.close()

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass
