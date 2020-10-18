#!/usr/bin/env python

import rospy
import json
import threading
import serial

from std_msgs.msg import String
from igvc_msgs.msg import motors, velocity, gps

# ROS node that facilitates all serial communications within the robot
# Subscribes to motor values
# Publishes GPS coordinates

serials = {}

class VelocitySerialReadThread(threading.Thread):
    def __init__(self, serial_obj, topic):
        threading.Thread.__init__(self)

        self.serial_obj = serial_obj

        # Allow timeout of up to 1 second on reads. This could be set to None to have infinite timeout,
        # but that would hault the node when it tries to exit. Need to make sure the while loop condition is
        # semi-regularly checked. This is better than rospy.Rate because it will continously wait for new message
        # instead of only checking on a fixed interval.
        self.serial_obj.timeout = 1

        # Assumes String type for now. This class will need to be adapted in the future for different message types.
        self.publisher = rospy.Publisher(topic, velocity, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            line = self.serial_obj.readline() # Assume all messages end in newline character. This is standard among SCR IGVC serial messages.

            if line:
                print(line)
                vels = line.split(",")
                velPkt = velocity()
                velPkt.leftVel = float(vels[0])
                velPkt.rightVel = float(vels[1])

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
            coord = self.serial_obj.readline() # Assume all messages end in newline character. This is standard among SCR IGVC serial messages.

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

    # JSON packet to be sent
    motion_pkt = {
        "motorLeft": data.left,
        "motorRight": data.right,
    }

    # Encode JSON string to bytes
    json_dump = json.dumps(motion_pkt, separators = (',', ':'))
    out = (json_dump + "\n").encode()

    serials["motor"].write(out)


# Initialize the serial node
# Node handles all serial communication within the robot (motor, GPS)
def init_serial_node():
    
    # Setup serial node
    rospy.init_node("serial_node", anonymous = False)

    # Setup motor serial and subscriber
    motor_serial = serials["motor"] = serial.Serial(port = '/dev/igvc-nucleo-120', baudrate = 115200)
    rospy.Subscriber('/igvc/motors_raw', motors, motors_out)
    
    motor_response_thread = VelocitySerialReadThread(serial_obj = serials["motor"], topic = '/igvc/velocity')
    motor_response_thread.start()

    # Setup GPS serial and publisher
    gps_serial = serials["gps"] = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600)

    gps_response_thread = GPSSerialReadThread(serial_obj = serials["gps"], topic = '/igvc/gps')
    gps_response_thread.start()
    
    # Wait for topic updates
    rospy.spin()

    # Close the serial ports when program ends
    motor_serial.close()
    gps_serial.close()

if __name__ == '__main__':
    try:
        init_serial_node()
    except rospy.ROSInterruptException:
        pass
