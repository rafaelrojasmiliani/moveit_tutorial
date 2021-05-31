#!/usr/bin/env python2
from plot_qt_widget import PlotWindow

import rospy
import sys
import numpy as np
from PyQt5.QtGui import QApplication

from moveit_msgs.msg import RobotTrajectory


class TrajectoryPlotter(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)

        self.window_size = 20
        self.values = np.zeros((self.window_size))
        self.index = 0

        rospy.init_node('visualizer', anonymous=True)
        self.subscriber = rospy.Subscriber(
            "plot_trajectory", RobotTrajectory, self.plot, queue_size=1)

    def plot(self, _msg):
        """ Plot
        """

        joint_trajectory_msg = _msg.joint_trajectory

        joint_pos = np.array(
            [point.positions for point in joint_trajectory_msg.points])
        joint_vel = np.array(
            [point.velocities for point in joint_trajectory_msg.points])
        joint_acc = np.array(
            [point.accelerations for point in joint_trajectory_msg.points])
        joint_eff = np.array(
            [point.effort for point in joint_trajectory_msg.points])
        time_spam = np.array([point.time_from_start.to_sec()
                              for point in joint_trajectory_msg.points])

        time_is_defined = time_spam.ndim == 1 and \
            time_spam.shape[0] > 1 and np.mean(time_spam) > 0.01
        for i, curve in enumerate([joint_pos, joint_vel, joint_acc, joint_eff]):
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
    window = TrajectoryPlotter()
    window.show()
    app.exec_()
