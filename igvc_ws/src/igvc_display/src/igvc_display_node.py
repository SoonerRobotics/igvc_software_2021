#!/usr/bin/env python3

import rospy
from igvc_msgs.msg import EKFState
from nav_msgs.msg import OccupancyGrid, Path

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel
from PyQt5.QtCore import QTimer

import copy
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

app = None

class IGVCPathCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(IGVCPathCanvas, self).__init__(fig)

class IGVCWindow(QMainWindow):
    def __init__(self):
        super(IGVCWindow, self).__init__()

        # Setup node
        rospy.init_node("igvc_display_node")

        # Subscribe to necessary topics
        rospy.Subscriber("/igvc_slam/local_config_space", OccupancyGrid, self.c_space_callback, queue_size=1)
        rospy.Subscriber("/igvc_ekf/filter_output", EKFState, self.ekf_callback)
        rospy.Subscriber("/igvc/local_path", Path, self.path_callback)

        # Setup window
        self.setObjectName('IGVC 21')
        self.setWindowTitle("SCR IGVC 21")
        self.showMaximized()

        # Add components to window
        self.path_canvas = IGVCPathCanvas(self, width=5, height=4, dpi=100)
        self.setCentralWidget(self.path_canvas)

        # Setup ROS await close timer
        self.timer = QTimer()
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.ros_await_close)
        self.timer.start()

        self.curMap = None
        self.lastEKF = None
        self.ekfAtMap = None
        self.path = None

    def draw(self):
        if self.curMap and self.lastEKF and self.path:
            self.path_canvas.axes.clear()

            self.path_canvas.axes.imshow(np.rot90(np.transpose(np.reshape(self.curMap, (200, 200))), 2), interpolation = 'nearest', extent=[-10, 10, -10, 10])
            
            for pose in self.path.poses:
                point = (pose.pose.position.x, pose.pose.position.y)
                self.path_canvas.axes.plot(-point[1], point[0], '.', markersize=8, color="red")

            robot_pos = (self.lastEKF.x - self.ekfAtMap[0], self.lastEKF.y - self.ekfAtMap[1])
            self.path_canvas.axes.plot(robot_pos[1], -robot_pos[0], '.', markersize=16, color="black")

            self.path_canvas.draw()

    def ros_await_close(self):
        if rospy.is_shutdown():
            # Cleanup with ROS is done
            app.quit()

    def ekf_callback(self, data):
        self.lastEKF = data
        self.draw()

    def c_space_callback(self, data):
        self.curMap = data.data
        self.ekfAtMap = (self.lastEKF.x, self.lastEKF.y)

    def path_callback(self, data):
        self.path = data

# Main setup
if __name__ == '__main__':
    try:
        app = QApplication([])
        igvc_window = IGVCWindow()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
