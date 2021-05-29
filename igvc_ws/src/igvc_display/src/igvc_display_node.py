#!/usr/bin/env python3

import rospy
from igvc_msgs.msg import EKFState
from nav_msgs.msg import OccupancyGrid, Path

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel
from PyQt5.QtCore import QTimer

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
        rospy.Subscriber("/igvc/global_path", Path, self.path_callback)

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

    def ros_await_close(self):
        if rospy.is_shutdown():
            # Cleanup with ROS is done
            app.quit()

    def ekf_callback(self, data):
        self.lastEKF = data

    def c_space_callback(self, data):
        self.curEKF = self.lastEKF
        self.curMap = data

    def path_callback(self, data):
        self.path_canvas.axes.imshow(np.random.normal(0.5, 0.2, (200, 200)), interpolation = 'nearest')
        self.path_canvas.draw()

# Main setup
if __name__ == '__main__':
    try:
        app = QApplication([])
        igvc_window = IGVCWindow()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
