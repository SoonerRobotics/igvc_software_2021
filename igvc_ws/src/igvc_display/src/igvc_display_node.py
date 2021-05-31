#!/usr/bin/env python3

import rospy
from igvc_msgs.msg import EKFState, velocity, gps
from nav_msgs.msg import OccupancyGrid, Path

import sys
import PyQt5 as Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

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
        rospy.Subscriber("/igvc/state", EKFState, self.ekf_callback)
        rospy.Subscriber("/igvc/local_path", Path, self.path_callback)
        rospy.Subscriber("/igvc/velocity", velocity, self.velocity_callback)
        rospy.Subscriber("/igvc/gps", gps, self.gps_callback)

        # Setup window
        self.setWindowTitle("SCR IGVC 21")
        self.showMaximized()

        # Add components to window
        self.centralWidget = QWidget()
        self.hlayout = QHBoxLayout()
        self.centralWidget.setLayout(self.hlayout)

        self.vlayout = QVBoxLayout()
        self.hlayout.addLayout(self.vlayout)

        self.path_canvas = IGVCPathCanvas(self, width=4, height=4, dpi=120)
        self.hlayout.addWidget(self.path_canvas)

        self.speed_label = QLabel(self)
        self.speed_label.setText(f"Speed: {0:0.01f}mph")
        self.speed_label.setFont(QFont('Arial', 28))

        self.wheels_label = QLabel(self)
        self.wheels_label.setText(f"Wheels: {0:0.01f}m/s, {0:0.01f}m/s")
        self.wheels_label.setFont(QFont('Arial', 18))

        self.pose_label = QLabel(self)
        self.pose_label.setText(f"Pose: ({0:0.01f}m, {0:0.01f}m, {0:0.01f}°)")
        self.pose_label.setFont(QFont('Arial', 18))

        self.gps_label = QLabel(self)
        self.gps_label.setText(f"GPS: dddd")
        self.gps_label.setFont(QFont('Arial', 18))

        self.vlayout.addWidget(self.speed_label)
        self.vlayout.addWidget(self.wheels_label)
        self.vlayout.addWidget(self.pose_label)
        self.vlayout.addWidget(self.gps_label)

        self.setCentralWidget(self.centralWidget)

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

            # Zoom into -5m to 5m
            
            for pose in self.path.poses:
                point = (pose.pose.position.x, pose.pose.position.y)
                self.path_canvas.axes.plot(-point[1], point[0], '.', markersize=8, color="red")

            robot_pos = (self.lastEKF.x - self.ekfAtMap[0], self.lastEKF.y - self.ekfAtMap[1])
            self.path_canvas.axes.plot(robot_pos[1], -robot_pos[0], '.', markersize=16, color="black")

            self.path_canvas.axes.set_xlim(-5, 5)
            self.path_canvas.axes.set_ylim(-5, 5)

            self.path_canvas.draw()

    def ros_await_close(self):
        if rospy.is_shutdown():
            # Cleanup with ROS is done
            app.quit()

    def velocity_callback(self, data):
        data.leftVel *= 2
        data.rightVel *= 2
        self.wheels_label.setText(f"Wheels: {data.leftVel:0.01f}m/s, {data.rightVel:0.01f}m/s")
        self.speed_label.setText(f"Speedometer: {(data.leftVel + data.rightVel) * 1.119:0.01f}mph")

    def gps_callback(self, data):
        if data.hasSignal:
            self.gps_label.setText(f"GPS:\n\t{data.latitude:0.07f}\n\t{data.longitude:0.07f}")
        else:
            self.gps_label.setText(f"GPS: No Signal")

    def ekf_callback(self, data):
        self.lastEKF = data
        self.pose_label.setText(f"Pose: ({data.x:0.01f}m, {data.y:0.01f}m, {data.yaw:0.01f}°)")
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
        app.setApplicationName("SCR IGVC 21")
        igvc_window = IGVCWindow()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
