#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler
import tf.transformations

def robot_program():

    th = TrajectoryHandler()

    start = th.start
    pose_l = Pose(position=Point(0.8, 0.2, 0.17),
                  orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(0.8, 0.05, 0.17),
                  orientation=from_euler(0.0, pi, 0.0))

    poses = [start, pose_l, pose_r]

    th.publish_marker_array([pose_l, pose_r])

    for pose in poses:
        th.sequencer.plan(Ptp(goal=pose, vel_scale=0.15, acc_scale=0.15))
        th.sequencer.execute()
        #print(tf.transformations.quaternion_from_euler(0.0, pi, 0.0))


if __name__ == '__main__':

    robot_program()
