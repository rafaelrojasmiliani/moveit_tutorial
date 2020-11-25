#!/usr/bin/python
import rospy
from moveit.msgs import RobotTrajectory
import numpy as np
import matplotlb.pyplot as plt


class cTrajectoryPlotter(object):
    """Add node description here"""

    def __init__(self):
        rospy.init_node('trajectory_plotter')
        rospy.Subscriber("plot_trajectory", RobotTrajectory,
                         self.plot, queue_size=1)
        rospy.spin()

        fig, ax = plt.subplots(4, 6)

        self.ax_ = ax
        self.fig_ = fig

    def plot(self, _msg):

        jt = _msg.joint_trajectory

        q = np.array([point.positions for point in jt.points])
        qd = np.array([point.velocities for point in jt.points])
        qdd = np.array([point.accelerations for point in jt.points])
        f = np.array([point.effort for point in jt.points])
        time_spam = np.array([point.time_from_start for point in jt.points])

        ax = self.ax_

        for i, curve in enumerate([q, qd, qdd, f]):
            for j in range(6):
                ax[i, j].plot(time_spam, curve[:, j])

        plt.show()


if __name__ == '__main__':
    plt.ion()
    cTrajectoryPlotter()
