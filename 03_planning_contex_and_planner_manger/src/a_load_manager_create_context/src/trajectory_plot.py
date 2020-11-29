#!/usr/bin/env python2
from plot_helper import PlotWindow

import rospy
import sys
import random
import numpy as np
from PyQt4.QtGui import QApplication

import numpy
from moveit_msgs.msg import RobotTrajectory


class cTrajectoryPlot(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)

        self.window_size = 20
        self.values = numpy.zeros((self.window_size))
        self.index = 0

        rospy.init_node('visualizer', anonymous=True)
        self.subscriber = rospy.Subscriber(
            "plot_trajectory", RobotTrajectory, self.plot, queue_size=1)

    def plot(self, _msg):

        jt = _msg.joint_trajectory

        q = np.array([point.positions for point in jt.points])
        qd = np.array([point.velocities for point in jt.points])
        qdd = np.array([point.accelerations for point in jt.points])
        f = np.array([point.effort for point in jt.points])
        time_spam = np.array([point.time_from_start.to_sec()
                              for point in jt.points])

        time_is_defined = time_spam.ndim == 1 and \
            time_spam.shape[0] > 1 and np.mean(time_spam) > 0.01
        for i, curve in enumerate([q, qd, qdd, f]):
            curve_is_defined = curve.ndim == 2 and \
                curve.shape[0] > 1 and curve.shape[1] == 6
            for j in range(6):
                self.axes[i, j].clear()
                self.axes[i, j].set_autoscaley_on(False)
                self.axes[i, j].set_ylim([-7, 7])
                if curve_is_defined and time_is_defined:
                    self.axes[i, j].plot(time_spam, curve[:, j], 'bo-')
                elif curve_is_defined:
                    self.axes[i, j].plot(curve[:, j], 'bo-')

        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = cTrajectoryPlot()
    window.show()
    app.exec_()
