#!/usr/bin/env python3

import rospy
import json
import threading
import serial
import can
import struct

from std_msgs.msg import String, Bool
from igvc_msgs.msg import motors, velocity, gps

# ROS node that facilitates all serial communications within the robot
# Subscribes to motor values
# Publishes GPS coordinates

serials = {}
cans = {}

mob_publisher = rospy.Publisher("/igvc/mobstart", Bool, queue_size=1)

MAX_SPEED = 2.2 # m/s
CAN_ID_ESTOP = 0
CAN_ID_MOBSTOP = 1
CAN_ID_MOBSTART = 9
CAN_ID_SEND_VELOCITY = 10
CAN_ID_RECV_VELOCITY = 11

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
            
            if msg is None:
                print("Received None CAN msg")
                continue

            if msg:
                if msg.arbitration_id == CAN_ID_RECV_VELOCITY:
                    left_speed, right_speed, max_speed = struct.unpack("bbB", msg.data)

                    velPkt = velocity()
                    velPkt.leftVel = left_speed / 127 * max_speed / 10
                    velPkt.rightVel = -right_speed / 127 * max_speed / 10

                    # print(f"Received {velPkt.leftVel} {velPkt.rightVel}")

                    self.publisher.publish(velPkt)
                
                if msg.arbitration_id == CAN_ID_ESTOP or msg.arbitration_id == CAN_ID_MOBSTOP:
                    # Stop blinking
                    serials["gps"].write(b'n')

                if msg.arbitration_id == CAN_ID_MOBSTART:
                    # Start blinking
                    serials["gps"].write(b'b')
                    mob_publisher.publish(Bool(True))


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
                    coord_msg.hasSignal = coord_json['hasSignal']

                    if coord_msg.hasSignal:
                        coord_msg.latitude = coord_json['latitude']
                        coord_msg.longitude = coord_json['longitude']

                    self.publisher.publish(coord_msg)
            except ValueError:
                pass

def clamp(val, min, max):
    if val < min:
        return min
    if val > max:
        return max
    return val

# Constructs motor message from given data and sends to serial
def motors_out(data):

    # Soon to be firmware corrections
    data.right = -data.right

    left_speed = clamp(int(data.left / MAX_SPEED * 127), -128, 127)
    right_speed = clamp(int(data.right / MAX_SPEED * 127), -128, 127)

    packed_data = struct.pack('bbB', left_speed, right_speed, int(MAX_SPEED * 10))

    can_msg = can.Message(arbitration_id=CAN_ID_SEND_VELOCITY, data=packed_data)

    try:
        cans["motor"].send(can_msg)
    except:
        print("Could not send CAN message")
    # else:
    #     print("CAN message sent!")

    # print(f"Sent {data.left} {data.right}")


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
    gps_serial = serials["gps"] = serial.Serial(port = '/dev/igvc-nucleo-722', baudrate = 9600)

    gps_response_thread = GPSSerialReadThread(serial_obj = serials["gps"], topic = '/igvc/gps')
    gps_response_thread.start()
    
    # Wait for topic updates
    rospy.spin()

    # Close the serial ports when program ends
    print("Closing threads")
    motor_can.close()
    gps_serial.close()

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass
