import can
import time

# Candlelight firmware on Linux
#bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

# Stock slcan firmware on Linux
# bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=100000)

# Stock slcan firmware on Windows
<<<<<<< HEAD
bus = can.interface.Bus(bustype='slcan', channel='COM7', bitrate=100000)
=======
bus = can.interface.Bus(bustype='slcan', channel='/dev/igvc-can-835', bitrate=100000)
>>>>>>> 7af85f818cdfcbb126dbfa3edf15a500e7282504

msg = can.Message(arbitration_id=0xc0ffee,
                  data=[0, 25, 0, 1, 3, 1, 4, 1],
                  is_extended_id=True)
while True:
    speed_left = eval(input())
    print (speed_left)

    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT received")