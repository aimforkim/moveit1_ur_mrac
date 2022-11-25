#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler
import tf.transformations

def robot_program():

    th = TrajectoryHandler()

    # start = th.start
    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)

    #start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    pose_l = Pose(position=Point(0.75, 0.2, 0.2),
                  orientation=Quaternion(0.0, 1.0, 0.0, 0.0))
    pose_r = Pose(position=Point(0.75, -0.2, 0.2),
                  orientation=Quaternion(0.0, 1.0, 0.0, 0.0))

    #poses = [start]
    poses = [start]

    th.publish_marker_array([pose_l, pose_r])

    for pose in poses:
        th.sequencer.plan(Ptp(goal=pose, vel_scale=0.2, acc_scale=0.2))
        th.sequencer.execute()


if __name__ == '__main__':

    robot_program()
