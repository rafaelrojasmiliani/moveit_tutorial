#!/usr/bin/env python2
import rospy
from moveit_msgs.msg import RobotTrajectory
import numpy as np
import matplotlib.pyplot as plt

global fig
global ax


def plot(_msg):
    global fig
    global ax

    jt = _msg.joint_trajectory

    q = np.array([point.positions for point in jt.points])
    qd = np.array([point.velocities for point in jt.points])
    qdd = np.array([point.accelerations for point in jt.points])
    f = np.array([point.effort for point in jt.points])
    time_spam = np.array([point.time_from_start.to_sec()
                          for point in jt.points])

    use_time = True
    if time_spam.ndim != 1 or\
            time_spam.shape[0] == 1 or\
            time_spam[-1]-time_spam[0] < 0.01:
        print('Time stamp ill defined. shape = ', time_spam.shape)
        use_time = False
    curve_name = ['position', 'velocity', 'acceleration', 'effort']

    for i, curve in enumerate([q, qd, qdd, f]):
        for j in range(6):
            if curve.ndim != 2 or curve.shape[0] == 1 or curve.shape[1] != 6:
                print('Curve '+curve_name[i]+' il defined', curve.shape)
            else:
                if use_time:
                    ax[i, j].plot(time_spam, curve[:, j], 'bo-')
                else:
                    ax[i, j].plot(curve[:, j], 'bo-')

    plt.draw()
    plt.pause(0.5)


if __name__ == '__main__':
    global fig
    global ax
    fig, ax = plt.subplots(4, 6)
    rospy.init_node("plotter")
    rospy.Subscriber("plot_trajectory", RobotTrajectory, plot)
    plt.ion()
    plt.show()
    rospy.spin()
