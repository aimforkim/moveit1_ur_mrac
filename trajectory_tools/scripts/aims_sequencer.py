#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler
import tf.transformations

def robot_program():

    th = TrajectoryHandler()

    start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    # pose_l = Pose(position=Point(0.8, 0.2, 0.17),
    #               orientation=from_euler(0.0, pi, 0.0))
    # pose_r = Pose(position=Point(0.8, 0.05, 0.17),
    #               orientation=from_euler(0.0, pi, 0.0))

    #start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    pose_l = Pose(position=Point(0.8, 0.4, 0.2),
                  orientation=Quaternion(0.0, 1.0, 0.0, 0.0))
    pose_r = Pose(position=Point(0.8, -0.4, 0.2),
                  orientation=Quaternion(0.0, 1.0, 0.0, 0.0))

    poses = [start]
    #poses = [start, pose_l, pose_r, start]

    th.publish_marker_array([pose_l, pose_r])

    for pose in poses:
        th.sequencer.plan(Ptp(goal=pose, vel_scale=0.15, acc_scale=0.15))
        th.sequencer.execute()


if __name__ == '__main__':

    robot_program()
